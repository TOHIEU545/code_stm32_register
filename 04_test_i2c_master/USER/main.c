/************************************************************
 * Project   : I2C Master Transmission (Normal & DMA)
 * Board     : STM32F103C8T6 (Blue Pill)
 * Function  :
 *   - Send data via I2C in Master mode with two options:
 *       0. Normal: CPU writes each byte to I2C1->DR.
 *       1. DMA: DMA1_Channel6 automatically transfers
 *          the whole buffer from RAM to I2C1->DR, 
 *          reducing CPU load.
 *
 * Hardware  :
 *   - I2C1: PB6 (SCL), PB7 (SDA), Slave address = 0x27.
 *   - Speed: 100 kHz (PCLK1=36 MHz, CCR=180, TRISE=37).
 *
 * Note      :
 *   - In DMA mode, ITBUFEN must be disabled to avoid TXE interrupts.
 *   - CCR calculation (Standard mode, 100 kHz):
 *        CCR = Fpclk1 / (2 * I2C_speed)
 *        Example: CCR = 36 MHz / (2 * 100 kHz) = 180
 ************************************************************/

#include "stm32f10x.h"
#include "Delay.h"
#include "Gpio.h"
#include "string.h"

#define USE_DMA 1                  //0: Normal, 1: Use DMA

// Function prototypes
void Cauhinh_Gpio(void);
void Cauhinh_I2C(void);
void Cauhinh_DMA(void);
void I2C1_Start(void);
void I2C1_SendAdd(uint8_t address);
void I2C1_SendData(uint8_t data);
void I2C1_SendString(char* _str);
void Tranfer_for_DMA(char* _str);
void I2C1_Stop(void);

int main(){
	Cauhinh_Gpio();
	Cauhinh_I2C();
	
	#if USE_DMA
		// Configure DMA if using DMA mode
		Cauhinh_DMA();
	#endif
	
	while(1){
		I2C1_Start();                      // Generate START condition
		I2C1_SendAdd(0x27 << 1);           // Send slave address (0x27) + write bit
		
		#if USE_DMA
			Tranfer_for_DMA("Use DMA\n");    // Transfer string via DMA
		#else
		
		I2C1_SendString("SPI on STM32\n"); // Transfer string in normal mode
		#endif
		
		I2C1_Stop();                       // Generate STOP condition
		Delay_Sys(2000);
	}
}

void Cauhinh_Gpio(void){
	Gpio_Config(GPIOB, GPIO_Pin_6, GPIO_Mode_AF_OD, GPIO_Speed_50MHz);
	Gpio_Config(GPIOB, GPIO_Pin_7, GPIO_Mode_AF_OD, GPIO_Speed_50MHz);
}

void Cauhinh_I2C(void){
	RCC -> APB1ENR |= RCC_APB1ENR_I2C1EN;
	I2C1 -> CR1 &= ~I2C_CR1_PE;                       // Disable I2C before config  
	
	// Set FREQ = 36 MHz (PCLK1)
	I2C1 -> CR2 &= ~I2C_CR2_FREQ;
	I2C1 -> CR2 |= (I2C_CR2_FREQ_2 | I2C_CR2_FREQ_5);
	
	I2C1 -> CCR &= ~I2C_CCR_FS;                       // Standard mode
	I2C1 -> CCR = 180;                                // CCR value for 100 kHz
	
	I2C1 ->TRISE = 37;
	
	I2C1 ->CR2 |= I2C_CR2_DMAEN;                       // Enable DMA requests
	
	I2C1 -> CR1 |= I2C_CR1_PE;
}

// Configure DMA1 Channel6 for I2C1_TX
void Cauhinh_DMA(void){
	RCC -> AHBENR |= RCC_AHBENR_DMA1EN;
	
	DMA1_Channel6 -> CCR &= ~DMA_CCR6_EN;
	
	DMA1_Channel6 -> CPAR = (uint32_t)&I2C1 -> DR;
	DMA1_Channel6 -> CCR |= DMA_CCR6_PL_0;
	DMA1_Channel6 -> CCR &= ~DMA_CCR6_MSIZE;
	DMA1_Channel6 -> CCR &= ~DMA_CCR6_PSIZE;
	DMA1_Channel6 -> CCR |= DMA_CCR6_MINC;
	DMA1_Channel6 -> CCR |= DMA_CCR6_DIR;
}

void I2C1_Start(void){
	I2C1 -> CR1 |= I2C_CR1_START;                  // Request START
	while(!(I2C1 -> SR1 & I2C_SR1_SB));            // Wait until START sent
	(void) I2C1 -> SR1;
} 

void I2C1_SendAdd(uint8_t address){
	I2C1 -> DR = address;                          // Send address
	while(!(I2C1 -> SR1 & I2C_SR1_ADDR));          // Wait for ADDR flag
	
	// Read SR1 and SR2 to clear ADDR
	(void)I2C1 -> SR1;
	(void)I2C1 -> SR2;
}

void I2C1_SendData(uint8_t data){
	while(!(I2C1 -> SR1 & I2C_SR1_TXE));
	I2C1 -> DR = data;
}

void I2C1_SendString(char* _str){
	while(*_str){
		I2C1_SendData(*_str++);
	}
}

void Tranfer_for_DMA(char* _str){
	DMA1_Channel6 -> CNDTR = strlen(_str);
	DMA1_Channel6 -> CMAR = (uint32_t)_str;
	DMA1_Channel6 -> CCR |= DMA_CCR6_EN;
	
	while(!(DMA1 -> ISR & DMA_ISR_TCIF6));
	DMA1 -> IFCR |= DMA_IFCR_CTCIF6;
	
	while(!(I2C1 -> SR1 & I2C_SR1_BTF));             // Wait until last byte transfer finishes
	
	DMA1_Channel6 -> CCR &= ~DMA_CCR6_EN;            // Disable DMA channel
}

void I2C1_Stop(void){
	while(!(I2C1 -> SR1 & I2C_SR1_BTF));             // Wait until transfer finished
	I2C1 -> CR1 |= I2C_CR1_STOP;                      // Send STOP condition
}
