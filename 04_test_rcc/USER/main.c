#include "stm32f10x.h"
#include "Gpio.h"
#include "Delay.h"

#define SYSCLK_MODE_HSE     0
#define SYSCLK_MODE_PLL     1

#define SYSCLK_MODE         SYSCLK_MODE_PLL

#define PLL_SRC_HSE         0
#define PLL_SRC_HSI_DIV2    1

#define PLL_SOURCE          PLL_SRC_HSE

void System_Clear(void);
void SetSystem_Clock(void);

int main()
{
	Gpio_Config(GPIOB, GPIO_Pin_12, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
	System_Clear();
	SetSystem_Clock();
	while(1)
	{
		GPIOB -> ODR ^= 1 << 12;
		Delay_Sys(100);
	}
}

void System_Clear(void)
{
	RCC -> CR |= (uint32_t)0x00000001;     // Set HSION
	RCC -> CFGR &= (uint32_t)0xF8FF0000;   // Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits
	RCC -> CR &= (uint32_t)0xFEF6FFFF;     // Reset HSEON, CSSON, PLLON
	RCC -> CR &= (uint32_t)0xFFFBFFFF;     // Reset HSEBYP bit
	RCC -> CFGR &= (uint32_t)0xFF80FFFF;   // Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE
	
	RCC->CIR = 0x009F0000;                 // Disable all interrupts and clear pending bits
}

void SetSystem_Clock(void){
	RCC -> CR |= RCC_CR_HSEON;
	
	while(!(RCC -> CR & RCC_CR_HSERDY));
	
	//******** Don't care ************//
	FLASH->ACR |= FLASH_ACR_PRFTBE;
	FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
  FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;
	//*******************************//
	
	RCC -> CFGR |= RCC_CFGR_HPRE_DIV1;               // HCLK = SYSCLK
	RCC -> CFGR |= RCC_CFGR_PPRE1_DIV2;              // PCLK1 = HCLK / 2 
	RCC -> CFGR |= RCC_CFGR_PPRE2_DIV1;              // PClK2 = HCLK
	
	#if SYSCLK_MODE == SYSCLK_MODE_PLL
	
		#if PLL_SOURCE == PLL_SRC_HSE
			RCC -> CFGR |= RCC_CFGR_PLLSRC_HSE;
			RCC -> CFGR |= RCC_CFGR_PLLXTPRE_HSE;
		#elif PLL_SOURCE == PLL_SRC_HSI_DIV2
			RCC -> CFGR |= RCC_CFGR_PLLSRC_HSI_Div2;
		#endif
		
		RCC ->CFGR |= RCC_CFGR_PLLMULL9;
		
		RCC -> CR |= RCC_CR_PLLON;
		while(!(RCC -> CR & RCC_CR_PLLRDY));
		
		RCC -> CFGR |= RCC_CFGR_SW_PLL;
		while((RCC -> CFGR & RCC_CFGR_SWS) != 0x08);

	#elif SYSCLK_MODE == SYSCLK_MODE_HSE
		
		RCC -> CFGR |= RCC_CFGR_SW_HSE;
		while((RCC -> CFGR & RCC_CFGR_SWS) != 0x04);
	
	#endif
}
