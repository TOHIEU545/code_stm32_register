/************************************************************
 * Project      : UART Communication (Normal Mode, DMA, and Hardware Flow Control)
 * Board        : STM32F103C8T6 (Blue Pill)
 * Description  :
 *   This program demonstrates three methods for transmitting
 *   data via USART1 on STM32:
 *
 *     0. Normal Mode:
 *        - Polling-based transmission using TXE flag
 *     1. DMA Mode:
 *        - Data is transferred from memory to USART1->DR 
 *          automatically via DMA1 Channel 4
 *     2. Hardware Flow Control Mode (RTS/CTS):
 *        - CTS (PA11) is monitored; data is sent only when
 *          the connected device asserts RTS (LOW state)
 *        - Transmission handled by USART1 interrupts
 *
 *   Transmission mode is selected via USE_Mode macro:
 *        USE_Mode = 0 ? Normal
 *        USE_Mode = 1 ? DMA
 *        USE_Mode = 2 ? Hardware Flow Control (HIGH -> LOW )
 *
 * Hardware Connections:
 *   - USART1 TX: PA9
 *   - USART1 RX: PA10
 *   - CTS: PA11 (only used in Hardware Flow Control mode)
 *   - Connect to an external UART device or loopback
 *
 * Notes:
 *   - Baud rate is fixed at 9600 bps (PSC set for 72 MHz system clock)
 *   - DMA mode uses USART1 TX DMA request
 *   - Hardware Flow Control mode requires proper RTS/CTS wiring
 ************************************************************/


#include "stm32f10x.h"
#include "Gpio.h"
#include "Timer.h"
#include "stdio.h"
#include "Debug.h"
#include "Delay.h"

#define USE_Mode 0           // 0: Use Normal send, 1: Use DMA, 2: Use Hardware              

#if USE_Mode == 2
	uint8_t sending_in_progress = 0;
	uint8_t send_index = 0;
	char data_to_send_by_Hardware[100] = "Use HardWare Flow Control\n";      // Data for interrupt transmission
#endif

// Function prototypes
void Cauhinh_Uart(void);
void Cauhinh_DMA(void);
void Cauhinh_NVIC(void);
void USART1_IRQHandler(void);
void Uart_SendChar(uint8_t char_);
void Uart_SendString(const char* string_);
void Tranfer_for_DMA(char* _str);


int main(){
	// Configure TX (PA9) and RX (PA10) pins for USART1
	Gpio_Config(GPIOA, GPIO_Pin_9, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	Gpio_Config(GPIOA, GPIO_Pin_10, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
	Cauhinh_Uart();
	
	#if USE_Mode == 1
		Cauhinh_DMA();
	#elif USE_Mode == 2
		Gpio_Config(GPIOA, GPIO_Pin_11, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);    // Configure CTS (PA11) if use Hardware Control
		Cauhinh_NVIC();
	#endif
	
	while(1){
		#if USE_Mode == 1
			Tranfer_for_DMA("Use DMA\n");
		#elif USE_Mode == 0
			Uart_SendString("Normal\n");
		#endif
		
		Delay_Sys(1000);    // Wait 1 second between transmissions
	}
}

// UART1 configuration
void Cauhinh_Uart(void){
	RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;                // Enable USART1 clock
	
	USART1 -> CR1 = 0;                                     // Reset control register
	USART1 -> CR2 = 0;
	USART1 -> CR3 = 0;
	
	USART1 -> BRR = 72000000 / 9600;                       // Baudrate = 9600
	USART1 -> CR1 &= ~USART_CR1_M;                         // 8 bit data
	USART1 -> CR2 &= ~USART_CR2_STOP;                      // 1 stop bit
	USART1 -> CR1 &= ~USART_CR1_PCE;                       // No parity
	USART1 -> CR1 |= USART_CR1_TE;                         // Enable trans
	
	#if USE_Mode == 1
		USART1 -> CR3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE);   // Disable hardware control
		USART1 -> CR3 |= USART_CR3_DMAT;                       // Enable DMA trans
	#elif USE_Mode == 2
		USART1 -> CR3 |= (USART_CR3_RTSE | USART_CR3_CTSE);   // Enable hardware control
		USART1 -> CR3 |= USART_CR3_CTSIE;      
	#elif USE_Mode == 0
		USART1 -> CR3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE);   // Disable hardware control
	#endif
	
	USART1 -> CR1 |= USART_CR1_UE;                         // Enable USART
}

// DMA configuration for USART1 TX (Channel 4)
void Cauhinh_DMA(void){
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;                      // Enable DMA1 clock 
	
	DMA1_Channel4 -> CCR &= ~DMA_CCR4_EN;                  // Disable DMA before config
	
	DMA1_Channel4 -> CPAR = (uint32_t)&USART1 -> DR;       // Add: USART1->DR
	DMA1_Channel4 -> CCR |= DMA_CCR4_PL_0;                 // Medium priority
	DMA1_Channel4 -> CCR &= ~DMA_CCR4_MSIZE;               // Memory size 8 bit
	DMA1_Channel4 -> CCR |= DMA_CCR4_PSIZE_0;              // Peripheral size 16 bit
	DMA1_Channel4 -> CCR |= DMA_CCR4_MINC;                 // Auto-increment memory pointer
	DMA1_Channel4 -> CCR |= DMA_CCR4_DIR;                  // Read from memory
}

// NVIC configuration for USART1 interrupt (optional)
void Cauhinh_NVIC(void){
	NVIC_InitTypeDef nvic;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	nvic.NVIC_IRQChannel = USART1_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	
	NVIC_Init(&nvic);
}

// USART1 interrupt handler (used for interrupt-based sending) if ON bit CTSIE
void USART1_IRQHandler(void){
	#if USE_Mode == 2
		// Check if CTS (Clear To Send)
		if(USART1 -> SR & USART_SR_CTS){
			// Check if the CTS input pin (PA11) is LOW (RTS)
			if((GPIOA -> IDR & GPIO_Pin_11) == 0){
				// If not already sending
				if(sending_in_progress == 0){
					send_index = 0;
					sending_in_progress = 1;
					USART1 -> CR1 |= USART_CR1_TXEIE;        // Enable TXE interrupt to start sending
				}
			}
			USART1 -> SR &= ~USART_SR_CTS;               // Clear CTS interrupt flag
		}
		
		// Check if TXE interrupt is triggered and TXEIE is enabled
		if((USART1 -> SR & USART_SR_TXE) && (USART1 -> CR1 & USART_CR1_TXEIE)){
			// If there are still characters left to send
			if (data_to_send_by_Hardware[send_index] != '\0'){
				USART1 -> DR = data_to_send_by_Hardware[send_index++];
			}
			else{
				USART1->CR1 &= ~USART_CR1_TXEIE;           // Disable TXE interrupt (transmission done)
				sending_in_progress = 0;
			}
		}
	#endif
}

void Uart_SendChar(uint8_t char_){
	while(!(USART1 -> SR & USART_SR_TXE));
	USART1 -> DR = char_;
}

void Uart_SendString(const char* string_){
	while(*string_){
		Uart_SendChar(*string_++);
	}
}

// Start DMA transmission for USART1
void Tranfer_for_DMA(char* _str){
	DMA1_Channel4 -> CNDTR = strlen(_str);                // Set number of bytes to send
	DMA1_Channel4 -> CMAR = (uint32_t)_str;               // Add: data
	DMA1_Channel4 -> CCR |= DMA_CCR4_EN;                  // Enable DMA
	
	while(!(DMA1 -> ISR & DMA_ISR_TCIF4));                // Wait for DMA transfer complete
	DMA1 -> IFCR |= DMA_IFCR_CTCIF4;                      // Clear transfer complete flag
	
	while(!(USART1 -> SR & USART_SR_TC));                  // Wait for USART to finish transmission
	USART1 -> SR &= ~USART_SR_TC;                          // Clear TC flag
	
	DMA1_Channel4 -> CCR &= ~DMA_CCR4_EN;                  // Disable DMA channel
}
