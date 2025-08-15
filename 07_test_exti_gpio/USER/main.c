/***************************************************************
 * Project: NVIC Configuration Example on STM32F10x
 *
 * Description:
 * This example demonstrates two different methods for configuring
 * the Nested Vectored Interrupt Controller (NVIC) on an STM32F10x MCU.
 *
 * --------------------------------------------------------------------
 * Method 1: Direct NVIC Configuration (Cauhinh_NVIC)
 * --------------------------------------------------------------------
 * - Sets the priority grouping directly in SCB->AIRCR.
 * - Enables the desired IRQ channel in NVIC->ISER.
 * - Sets the interrupt priority in NVIC->IP.
 * - All configuration is done in one function.
 * 
 *
 * --------------------------------------------------------------------
 * Method 2: Custom NVIC Configuration (Custom API style like misc.h)
 * --------------------------------------------------------------------
 * - Step 1: NVIC_SetPriorityGrouping_Custom(priorityGroup)
 *           Configures the priority grouping (preemption/subpriority split).
 *
 * - Step 2: NVIC_ConfigChannel_Custom(irqChannel, preemptionPriority, subPriority)
 *           Enables the IRQ channel and sets its preemption and subpriority.
 *
 *
 * --------------------------------------------------------------------
 * Hardware setup for this example:
 * --------------------------------------------------------------------
 * - PA8 configured as input pull-up, connected to an external button.
 * - EXTI8 line triggers on falling edge of PA8.
 * - PC13 configured as output push-pull for an LED (optional feedback).
 *
 * --------------------------------------------------------------------
 * Interrupt behavior:
 * --------------------------------------------------------------------
 * - Each time the button on PA8 is pressed (falling edge),
 *   the EXTI9_5_IRQHandler is triggered and increments the 'count' variable.
 *
 ***************************************************************/


#include "stm32f10x.h"
#include "Gpio.h"
#include "Delay.h"

#define Method 2

static uint8_t count = 0;

void Cauhinh_Gpio(void);
void Cauhinh_EXTI(void);
void Cauhinh_NVIC(void);
void NVIC_SetPriorityGrouping_Custom(uint32_t priorityGroup);
void NVIC_ConfigChannel_Custom(uint8_t irqChannel, uint8_t preemptionPriority, uint8_t subPriority);
void EXTI9_5_IRQHandler(void);

int main(){ 
	Cauhinh_Gpio();
	Cauhinh_EXTI();
	#if Method == 1
		Cauhinh_NVIC();
	#elif Method == 2
		NVIC_SetPriorityGrouping_Custom(NVIC_PriorityGroup_1);
		NVIC_ConfigChannel_Custom(EXTI9_5_IRQn, 0, 1);
	#endif
	while(1)
	{
		
	}
}

void Cauhinh_Gpio(void){
	// CLK for port A
	RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	
	// Configure PA8 as input pull-up for EXTI8
	GPIOA -> CRH &= (uint32_t)~0x0f;
	GPIOA -> CRH &= ~GPIO_CRH_MODE8;
	GPIOA -> CRH |= GPIO_CRH_CNF8_1;
	GPIOA -> ODR |= 1 << 8;
	
	// Configure PC13 as output push-pull, 50MHz (LED)
	Gpio_Config(GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
}

void Cauhinh_EXTI(void){
	// Enable clock for AFIO (Alternate Function I/O) for EXTI configuration
	RCC ->APB2ENR |= RCC_APB2ENR_AFIOEN;
	
	// Map EXTI8 line to PA8
	AFIO -> EXTICR[2] &= (uint32_t)~AFIO_EXTICR3_EXTI8;
	AFIO -> EXTICR[2] |= AFIO_EXTICR3_EXTI8_PA;
	
	// Enable interrupt mask for EXTI8
	EXTI -> IMR |= EXTI_IMR_MR8;                   // Enable interrupt request from line 8     
	EXTI -> EMR &= ~EXTI_EMR_MR8;                  // Disable event request
	
	// Configure interrupt trigger on falling edge
	EXTI -> RTSR &= ~EXTI_RTSR_TR8;                // Disable rising edge trigger
	EXTI -> FTSR |= EXTI_FTSR_TR8;                 // Enable falling edge trigger
}

void Cauhinh_NVIC(void){
	// Configure interrupt priority grouping
	uint32_t temp = SCB -> AIRCR;
	temp |= (0x5FA << 16) | SCB_AIRCR_PRIGROUP7;              // Write key 0x5FA and set priority group
	SCB -> AIRCR = temp;
	
	// Enable EXTI9_5 interrupt in NVIC
	NVIC -> ISER[EXTI9_5_IRQn >> 5] = NVIC_ISER_SETENA_23;
	
	// Set priority for EXTI9_5 interrupt
	NVIC -> IP[EXTI9_5_IRQn] = 1 << 4;
}

void NVIC_SetPriorityGrouping_Custom(uint32_t priorityGroup){
	// Set the priority grouping
	uint32_t temp = SCB -> AIRCR;
	temp |= (0x5FA << 16) | priorityGroup;
	SCB -> AIRCR = temp;
}

void NVIC_ConfigChannel_Custom(uint8_t irqChannel, uint8_t preemptionPriority, uint8_t subPriority){
	// Enable the IRQ channel
	NVIC -> ISER[irqChannel >> 5] = 1 << (irqChannel & 0x1f);
	
	// Variables for priority calculation
	uint32_t preemptBits     = 0x00;                         // Number of bits for preemption priority from group config
	uint32_t preemptShift    = 0x00;                         // Bit shift amount for preemption priority
	uint32_t subPriorityMask = 0x0F;                         // Bit mask for subpriority
	
	// Calculate preemption bits and masks based on current priority group
	preemptBits = (0x700 - (SCB -> AIRCR & 0x700)) >> 8;
	preemptShift = (0x4 - preemptBits);
	subPriorityMask = subPriorityMask >> preemptBits;
	
	// Combine preemption and subpriority into final priority value
	uint32_t PriorityValue = 0;
	PriorityValue = (uint32_t)preemptionPriority << preemptShift;
	PriorityValue |= (subPriority & subPriorityMask);
	PriorityValue = PriorityValue << 4;
	
	// Apply priority to NVIC
	NVIC -> IP[irqChannel] = (uint8_t)PriorityValue;
}

void EXTI9_5_IRQHandler(void){
	if(EXTI -> PR & EXTI_PR_PR8){
		count ++;
		EXTI -> PR = EXTI_PR_PR8;                    // Clear interrupt pending flag for EXTI8
	}
}
