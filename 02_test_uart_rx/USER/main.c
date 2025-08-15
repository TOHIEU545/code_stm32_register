/************************************************************
 * Project      : UART Reception via RXNE Interrupt & DMA + IDLE
 * Board        : STM32F103C8T6 (Blue Pill)
 * Description  :
 *   This program demonstrates two methods to receive data via
 *   STM32 USART1:
 *     1. RXNE interrupt-based reception
 *     2. DMA + IDLE line detection
 * 
 *   The user can choose either mode by commenting/uncommenting
 *   the appropriate configurations.
 *
 *   RX Pin: PA10
 *   TX Pin: PA9  (used for feedback/testing)
 * 
 *   Mode 1: RXNE Interrupt
 *     - Simple method, reads data byte-by-byte in interrupt.
 *     - Suitable for small and fixed-size data.
 *     - Requires CPU intervention on each byte.
 * 
 *   Mode 2: DMA + IDLE Line
 *     - Efficient for variable-length strings (with end char like '\n').
 *     - DMA receives data into buffer automatically.
 *     - USART IDLE interrupt triggers after the last byte.
 *     - CPU processes full buffer only once per message.
 *     - Suitable for large or frequent data streams.
 * 
 ************************************************************/

#include "stm32f10x.h"
#include "Gpio.h"
#include "Delay.h"
#include "Debug.h"

#define USE_DMA_RX 1   // 1: Use DMA + IDLE, 0: Use RXNE interrupt

// Global variables
static char str_received[100];                 
static char str_received_copy[100];
static uint8_t flag_received = 0, count_data = 0;

void cauhinhGPIO(void);
void Cauhinh_Uart_1(void);
void Cauhinh_DMA(void);
void cauhinhNVIC(void);
void USART1_IRQHandler(void);
void Cauhinh_Uart_2(void);
void Uart_2_SendChar(uint8_t char_);
void Uart_2_SendString(const char* string_);

int main(){
	cauhinhGPIO();
	Cauhinh_Uart_1();
	
	#if USE_DMA_RX
		Cauhinh_DMA();
	#endif
	
	cauhinhNVIC();
	Cauhinh_Uart_2();
	while(1){
		if(flag_received){
			#if USE_DMA_RX
				Uart_2_SendString(str_received_copy);
			#else
				Uart_2_SendString(str_received);
				Uart_2_SendChar('\n');
			#endif
			flag_received = 0;
		}
	}
}

// GPIO configuration
void cauhinhGPIO(void){
	// USART1: TX PA9, RX PA10
	Gpio_Config(GPIOA, GPIO_Pin_9, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	Gpio_Config(GPIOA, GPIO_Pin_10, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
	
	// USART2: TX PA2, RX PA3
	Gpio_Config(GPIOA, GPIO_Pin_2, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	Gpio_Config(GPIOA, GPIO_Pin_3, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
}

// USART1 configuration for reception
void Cauhinh_Uart_1(void){
	RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;

	USART1 -> CR1 = 0;
	USART1 -> BRR = 72000000 / 9600;
	USART1 -> CR1 &= ~USART_CR1_M;
	USART1 -> CR2 &= ~USART_CR2_STOP;
	USART1 -> CR1 &= ~USART_CR1_PCE;
	USART1 -> CR1 |= USART_CR1_RE; 
	
	#if USE_DMA_RX
		USART1 -> CR3 |= USART_CR3_DMAR;          // Enable DMA mode for reception
		USART1 -> CR1 |= USART_CR1_IDLEIE;        // Enable IDLE interrupt
	#else
		USART1 -> CR1 |= USART_CR1_RXNEIE;        // Enable RXNE interrupt
	#endif

	USART1 -> CR1 |= USART_CR1_UE;
}

// DMA1 Channel5 configuration for USART1_RX
void Cauhinh_DMA(void){
	RCC -> AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_Channel5 -> CCR &= (unsigned int )~DMA_CCR5_EN;
	
	DMA1_Channel5 -> CPAR = (uint32_t)&USART1 -> DR;
	DMA1_Channel5 -> CMAR = (uint32_t)&str_received;
	DMA1_Channel5 -> CNDTR = 100;
	DMA1_Channel5 ->CCR = 0;
	
	DMA1_Channel5 -> CCR |= DMA_CCR5_CIRC;          // Circular mode
	DMA1_Channel5 -> CCR &= (unsigned int )~DMA_CCR5_DIR;          // From UART1
	DMA1_Channel5 -> CCR |= DMA_CCR5_MINC;          // Memory increment 
	DMA1_Channel5 -> CCR |= DMA_CCR5_PSIZE_0;       
	DMA1_Channel5 -> CCR &= (unsigned int )~DMA_CCR5_MSIZE;
	DMA1_Channel5 -> CCR |= DMA_CCR5_PL_0;
	
	DMA1_Channel5 -> CCR |= DMA_CCR5_EN;
}

// NVIC configuration for USART1 interrupt
void cauhinhNVIC(void){
	NVIC_InitTypeDef nvic;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	nvic.NVIC_IRQChannel = USART1_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&nvic);
}

// USART1 interrupt handler
void USART1_IRQHandler(void){
	#if USE_DMA_RX
		if(USART1 -> SR & USART_SR_IDLE){
			(void)USART1 -> SR;
			(void)USART1 -> DR;
			
			DMA1_Channel5 -> CCR &= (unsigned int )~DMA_CCR5_EN;
			
			count_data = 100 - (uint8_t)DMA1_Channel5 -> CNDTR;
			str_received[count_data] = '\0';
			strcpy(str_received_copy, str_received);
			flag_received = 1;
			
			DMA1_Channel5 -> CNDTR = 100;
			DMA1_Channel5 -> CCR |= DMA_CCR5_EN;
		}
	#else
		if(USART1 -> SR & USART_SR_RXNE){
				char c = (char)(USART1 -> DR);
				if(c == '\n'){
					flag_received = 1;
					str_received[count_data] = '\0';
					count_data = 0;
				}
				else if(c != '\n'){
					str_received[count_data++] = c;
				}
			}
		#endif
}

// USART2 configuration for transmitting result
void Cauhinh_Uart_2(void){
	RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;
	
	USART2 -> CR1 = 0;
	USART2 -> BRR = 36000000 / 9600;
	USART2 -> CR1 &= ~USART_CR1_M;
	USART2 -> CR2 &= ~USART_CR2_STOP;
	USART2 -> CR1 &= ~USART_CR1_PCE;
	USART2 -> CR1 |= USART_CR1_TE;
	
	USART2 -> CR3 &= ~USART_CR3_CTSE;
	USART2 -> CR3 &= ~USART_CR3_RTSE;
	
	USART2 -> CR1 |= USART_CR1_UE;
}

// Send 1 character via USART2
void Uart_2_SendChar(uint8_t char_){
	while(!(USART2 -> SR & USART_SR_TXE));
	USART2 -> DR = char_;
}

// Send a string via USART2
void Uart_2_SendString(const char* string_){
	while(*string_){
		Uart_2_SendChar(*string_++);
	}
}
