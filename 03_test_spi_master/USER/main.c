/************************************************************
 * Project      : SPI Communication in Master Mode (3 NSS Modes) + DMA Support
 * Board        : STM32F103C8T6 (Blue Pill)
 * Description  :
 *   This program demonstrates SPI communication in Master Mode
 *   on STM32F103, with three selectable modes for controlling
 *   the NSS (Slave Select) signal, and optional DMA support
 *   for high-speed data transfer.
 *
 *   NSS Modes:
 *     0. Software NSS management: SSM = 1, SSI = 1
 *     1. Hardware NSS output:     SSM = 0, SSOE = 1
 *     2. Hardware NSS input:      SSM = 0, SSOE = 0 and NSS pin HIGH
 *
 *   - Mode 0: Master always enabled via software (no NSS pin).
 *   - Mode 1: NSS driven automatically by hardware when master transmits.
 *   - Mode 2: NSS monitored as input; MODF (Mode Fault) error is
 *             detected if NSS goes low unexpectedly.
 *
 *   DMA Support:
 *     - When USE_DMA = 1, SPI transmit is handled by DMA1_Channel5
 *       for minimal CPU load and maximum throughput.
 *     - DMA mode automatically transfers the specified buffer
 *       to the SPI data register (DR) without CPU intervention.
 *
 *   This example uses:
 *     - SPI2:  PB13 (SCK), PB14 (MISO), PB15 (MOSI), PB12 (NSS)
 *     - Optional UART1 for debug (Mode 2 only): PA9 (TX), PA10 (RX)
 *
 * ***********************************************************/


#include "stm32f10x.h"
#include "Delay.h"
#include "Gpio.h"
#include "Debug.h"

// Select NSS mode for SPI Master
// 0: SSM=1, SSI=1 (Software NSS control)
// 1: SSM=0, SSOE=1 (Hardware NSS output)
// 2: SSM=0, SSOE=0, NSS as input (detect MODF error)
#define USE_MODE_NSS  0

#define USE_DMA  1                            //1: Use DMA, 0: Normal

static uint8_t flag_error_modf = 0;          // Flag to indicate Mode Fault error

// Function prototypes
void cauhinhGpio(void);
void Cauhinh_SPI(void);
void Cauhinh_DMA(void);
void Cauhinh_NVIC(void);
void SPI2_IRQHandler(void);
void Spi_SendChar(char data);
void Spi_SendString(char* string_);
void Tranfer_for_DMA(char* _str);

int main() {
	cauhinhGpio();
	Cauhinh_SPI();
	
	#if USE_DMA
		Cauhinh_DMA();
	#endif
	
	#if USE_MODE_NSS == 2
		Uart_Config();           // Configure UART for MODF error debugging
		Cauhinh_NVIC();          // Configure SPI2 interrupt
	#endif

	while (1) {
		#if USE_MODE_NSS == 2
			 // If MODF error detected, print message and reset flag
			if(flag_error_modf == 1){
				printf("Error MODF\n");
				flag_error_modf = 0;
			}
		#endif
		
		#if USE_DMA
			Tranfer_for_DMA("SPI use DMA\n");
		#else	
		 Spi_SendString("SPI ON STM32\n");
		#endif
			
		Delay_Sys(1000);
	}
}

void cauhinhGpio(void) {
	#if USE_MODE_NSS == 1
		// Mode 1: NSS as hardware output, controlled by SPI peripheral
		Gpio_Config(GPIOB, GPIO_Pin_12, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	#elif USE_MODE_NSS == 2
		// Mode 2: NSS as input with pull-up resistor
		Gpio_Config(GPIOB, GPIO_Pin_12, GPIO_Mode_IPU, GPIO_Speed_50MHz);
	
		//Uart 1
		Gpio_Config(GPIOA, GPIO_Pin_9, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
		Gpio_Config(GPIOA, GPIO_Pin_10, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
	#endif
	// SPI2 Clock (PB13) - Alternate Function Push-Pull
	Gpio_Config(GPIOB, GPIO_Pin_13, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	// MOSI (PB15) - Alternate Function Push-Pull
	Gpio_Config(GPIOB, GPIO_Pin_15, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	// MISO (PB14) - Input floating
	Gpio_Config(GPIOB, GPIO_Pin_14, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
}

void Cauhinh_SPI(void){
	RCC -> APB1ENR |= RCC_APB1ENR_SPI2EN;                     // Enable SPI2 clock 
	SPI2 -> CR1 &= ~SPI_CR1_SPE;                              // Disable SPI before configuration
	 
	SPI2 -> CR1 &= ~SPI_CR1_BR;
	SPI2 -> CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0;// SPI speed = fPCLK/256
	
	// CPOL = 0, CPHA = 0 (SPI Mode 0)
	SPI2 -> CR1 &= ~SPI_CR1_CPOL;
	SPI2 -> CR1 &= ~SPI_CR1_CPHA;
	
	SPI2 -> CR1 &= ~SPI_CR1_DFF;                              // 8-bit data frame
	SPI2 -> CR1 &= ~SPI_CR1_LSBFIRST;                         // MSB first
	SPI2 -> CR1 |= SPI_CR1_MSTR;                              // Master mode
	
	SPI2 -> CR1 &= ~(SPI_CR1_BIDIMODE | SPI_CR1_RXONLY);      // Full-duplex mode
	
	#if USE_MODE_NSS == 1
		// Hardware NSS output mode
		SPI2 -> CR1 &= ~SPI_CR1_SSM;
		SPI2 -> CR2 |= SPI_CR2_SSOE;
	#elif USE_MODE_NSS == 2
		// Hardware NSS input mode (detect MODF)
		SPI2 -> CR1 &= ~SPI_CR1_SSM;
		SPI2 -> CR2 &= ~SPI_CR2_SSOE;
		
		// Enable error interrupt
		SPI2 -> CR2 |= SPI_CR2_ERRIE;
	#elif USE_MODE_NSS == 0
		// Software NSS management
		SPI2 -> CR1 |= SPI_CR1_SSM;
		SPI2 -> CR1 |= SPI_CR1_SSI;
	#endif
	
	#if USE_DMA
		SPI2 -> CR2 |= SPI_CR2_TXDMAEN;
	#endif
	
	SPI2 -> CR1 |= SPI_CR1_SPE;
}

void Cauhinh_DMA(void){
	RCC -> AHBENR |= RCC_AHBENR_DMA1EN;
	
	DMA1_Channel5 -> CCR &= ~DMA_CCR5_EN;
	
	DMA1_Channel5 -> CPAR = (uint32_t)&SPI2 -> DR;
	DMA1_Channel5 -> CCR |= DMA_CCR5_PL_0;
	DMA1_Channel5 -> CCR &= ~DMA_CCR5_MSIZE;
	DMA1_Channel5 -> CCR &= ~DMA_CCR5_PSIZE;
	DMA1_Channel5 -> CCR |= DMA_CCR5_MINC;
	DMA1_Channel5 -> CCR |= DMA_CCR5_DIR;
}

void Cauhinh_NVIC(void){
	NVIC_InitTypeDef nvic;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	nvic.NVIC_IRQChannel = SPI2_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&nvic);
}

void SPI2_IRQHandler(void){
	// Handle Overrun (OVR) error
	if(SPI2 -> SR & SPI_SR_OVR){
		(void)SPI2 -> DR;        // Read DR to clear OVR flag
		(void)SPI2 -> SR;        // Read SR to complete OVR clear
	}
	
	// Handle Mode Fault (MODF) error
	if(SPI2 -> SR & SPI_SR_MODF){
		flag_error_modf = 1;
		
		// Disable error interrupt to prevent repeated triggers
		SPI2 -> CR2 &= ~SPI_CR2_ERRIE;
	}
}

void Spi_SendChar(char data) {
	while(!(SPI2 -> SR & SPI_SR_TXE));       // Wait until TXE = 1 (data register empty)
	SPI2 -> DR = data;
}

void Spi_SendString(char* string_){
	// Send characters until NULL terminator is reached
	while(*string_){
		Spi_SendChar(*string_++);
	}
	while(SPI2 -> SR & SPI_SR_BSY);          // Wait until SPI is not busy (BSY = 0)
}

void Tranfer_for_DMA(char* _str){
	DMA1_Channel5 -> CNDTR = strlen(_str);
	DMA1_Channel5 -> CMAR = (uint32_t)_str;
	DMA1_Channel5 -> CCR |= DMA_CCR5_EN;
	
	while(!(DMA1 -> ISR & DMA_ISR_TCIF5));
	DMA1 -> IFCR |= DMA_IFCR_CTCIF5;
	
	while(SPI2 -> SR & SPI_SR_BSY);
	
	DMA1_Channel5 -> CCR &= ~DMA_CCR5_EN;
}
