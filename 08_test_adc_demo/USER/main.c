#include "stm32f10x.h"
#include "Gpio.h"
#include "Delay.h"
#include "Debug.h"
uint16_t adc_value = 0;

void Cauhinh_Gpio(void);
void Cauhinh_ADC1_Continuous_Mode(void);
void Cauhinh_ADC1_Single_Mode(void);
void ADC1_Single_Mode_Start(void);
void Cauhinh_DMA(void);
void Cauhinh_NVIC(void);
void ADC1_2_IRQHandler(void);

int main(){
	Uart_Config();
	Cauhinh_Gpio();
	Cauhinh_DMA();
	Cauhinh_ADC1_Continuous_Mode();
	Cauhinh_NVIC();
	while(1){
		printf("%u\n", adc_value);
		Delay_Sys(500);
	}
}

void Cauhinh_Gpio(void){
	Gpio_Config(GPIOA, GPIO_Pin_9, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	Gpio_Config(GPIOA, GPIO_Pin_10, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
	Gpio_Config(GPIOA, GPIO_Pin_0, GPIO_Mode_AIN, GPIO_Speed_50MHz);
	Gpio_Config(GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
}

void Cauhinh_ADC1_Continuous_Mode(void){
	RCC -> APB2ENR |= RCC_APB2ENR_ADC1EN;            // clk cho adc1
	
	ADC1 -> SQR1 &= ~ADC_SQR1_L;                     // Set the number of conversions in the regular sequence to 1 (L[3:0] = 0)
	ADC1 -> SQR3 &= ~ADC_SQR3_SQ1;                   // select channel 0 PA0
	ADC1 -> SQR3 |= 0;
	
	ADC1 -> CR1 &= ~ADC_CR1_SCAN;                    //Disable scan mode
	ADC1 -> CR2 |= ADC_CR2_CONT;                     //Enable continuous mode
	ADC1 -> CR2 |= ADC_CR2_DMA;                      //Enable DMA
	
	ADC1 -> CR1 |= ADC_CR1_AWDSGL;
	ADC1 -> CR1 &= ~ADC_CR1_AWDCH;
	ADC1 -> CR1 |= 0 << 0;
	ADC1 -> CR1 |= (ADC_CR1_AWDEN | ADC_CR1_AWDIE);
	ADC1 -> HTR = 3000;
	ADC1 -> LTR = 1000;
	
	ADC1 -> CR2 |= ADC_CR2_ADON;
	Delay_Sys_us(1);
	
	ADC1 -> CR2 |= ADC_CR2_CAL;                      //Start calib
	while(ADC1 -> CR2 & ADC_CR2_CAL);
	ADC1 -> CR2 |= ADC_CR2_ADON;
}

void Cauhinh_ADC1_Single_Mode(void){
	RCC -> APB2ENR |= RCC_APB2ENR_ADC1EN;            // clk cho adc1
	
	ADC1 -> SQR1 &= ~ADC_SQR1_L;                     // Set the number of conversions in the regular sequence to 1 (L[3:0] = 0)
	ADC1 -> SQR3 &= ~ADC_SQR3_SQ1;                   // select channel 0 PA0
	ADC1 -> SQR3 |= 0;
	
	ADC1 -> CR1 &= ~ADC_CR1_SCAN;                    //Disable scan mode
	ADC1 -> CR2 &= ~ADC_CR2_CONT;                    //Enable single mode
	ADC1->CR2 |= ADC_CR2_EXTSEL;                     // Selection trigger
	ADC1->CR2 |= ADC_CR2_EXTTRIG;
	ADC1 -> CR2 &= ~ADC_CR2_DMA;                     //Disable DMA
	
	
	ADC1 -> CR2 |= ADC_CR2_ADON;
	Delay_Sys_us(1);
	
	ADC1 -> CR2 |= ADC_CR2_CAL;                      //Start calib
	while(ADC1 -> CR2 & ADC_CR2_CAL);
	ADC1 -> CR2 |= ADC_CR2_ADON;
}

void ADC1_Single_Mode_Start(void){
	ADC1 -> CR2 |= ADC_CR2_SWSTART;
	while(!(ADC1 -> SR & ADC_SR_EOC));
	adc_value = (uint16_t)ADC1 -> DR;
}

void Cauhinh_DMA(void){
	RCC -> AHBENR |= RCC_AHBENR_DMA1EN;
	
	DMA1_Channel1 -> CCR &= ~DMA_CCR1_EN;
	
	DMA1_Channel1 -> CPAR = (uint32_t)&(ADC1 -> DR);
	DMA1_Channel1 -> CMAR = (uint32_t)&adc_value;
	DMA1_Channel1 -> CNDTR = 1;
	DMA1_Channel1 -> CCR |= DMA_CCR1_PL;
	DMA1_Channel1 -> CCR |= DMA_CCR1_CIRC;
	DMA1_Channel1 -> CCR |= (DMA_CCR1_PSIZE_0 | DMA_CCR1_MSIZE_0);
	
	DMA1_Channel1 -> CCR |= DMA_CCR1_EN;
}

void Cauhinh_NVIC(void){
	NVIC_InitTypeDef nvic;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	nvic.NVIC_IRQChannel = ADC1_2_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&nvic);
}

void ADC1_2_IRQHandler(void){
	if(ADC1 -> SR & ADC_SR_AWD)
	{
		printf("Warning!!!!\n");
		ADC1 -> SR &= ~ADC_SR_AWD;
	}
}
