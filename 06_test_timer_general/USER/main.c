/************************************************************
 * Project   : TIM2 Output (Compare / PWM) & TIM3 Input (Capture / PWM Input)
 * Board     : STM32F103C8T6 (Blue Pill)
 * Function  :
 *   - Output (TIM2, CH1, PA0):
 *       0. Output Compare: generate interrupt when Counter = CCR1 (interrupt test).
 *       1. PWM Output: generate PWM waveform with configurable frequency and duty cycle.
 *   - Input (TIM3, CH1 & CH2, PA6):
 *       0. Input Capture: measure signal frequency by capturing period between two rising edges.
 *       1. PWM Input: measure both signal frequency and duty cycle.
 *
 * Hardware  :
 *   - TIM2 CH1: PA0 (output).
 *   - TIM3 CH1/CH2: PA6 (input).
 *   - Connect TIM2 output pin to TIM3 input pin (jumper wire).
 *   - PA3: LED indicator in Output Compare mode.
 *
 * Note      :
 *   - MODE_OUTPUT selects output mode:
 *       0 = Output Compare, 1 = PWM Output.
 *   - MODE_INPUT selects input mode:
 *       0 = Input Capture, 1 = PWM Input.
 *   - TIM2 and TIM3 run independently; prescalers do not need to match.
 *   - Ensure proper connection between output and input pins for measurement.
 ************************************************************/

#include "stm32f10x.h"
#include "Gpio.h"
#include "Delay.h"

// ===== Mode Configuration =====
// MODE_OUTPUT: 0 = Output Compare mode, 1 = PWM Output mode
// MODE_INPUT : 0 = Input Capture mode, 1 = PWM Input mode
#define MODE_OUTPUT  1
#define MODE_INPUT   1

// ===== Global Variables =====
#if MODE_INPUT == 0
	// For Input Capture mode: store last rising-edge capture value
	static volatile uint32_t last_capture_rising = 0;
#elif MODE_INPUT == 1
	// For PWM Input mode: store measured duty cycle percentage
	static volatile uint32_t duty_cycle = 0;
#endif
static volatile uint32_t period = 0;           // Measured signal period (in timer ticks)
static volatile uint32_t frequency = 0;        // Measured signal frequency (Hz)

// ===== Function Prototypes =====
void Tim2_Output_Config(void);
void Tim3_Input_Config(void);
void Cauhinh_NVIC(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);

int main()
{
	// Tim2 CH1
	Gpio_Config(GPIOA, GPIO_Pin_0, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	// Tim3 CH1/CH2
	Gpio_Config(GPIOA, GPIO_Pin_6, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
	// PA3 Led blink
	Gpio_Config(GPIOA, GPIO_Pin_3, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
	
	Tim2_Output_Config();
	Tim3_Input_Config();
	Cauhinh_NVIC();
	while(1)
	{
		
	}
}

// ===== TIM2 Configuration =====
// Generates output signal in either Output Compare or PWM mode
void Tim2_Output_Config(void)
{
	// CLK for TIM2
	RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
	// Disable TIM2
	TIM2 -> CR1 &= ~TIM_CR1_CEN;
	
	// Mode up
	TIM2 -> CR1 &= ~ TIM_CR1_DIR;
	TIM2 -> PSC = 7199;
	TIM2 -> ARR = 5000 - 1;
	TIM2 -> EGR |= TIM_EGR_UG;           // Update registers immediately
	
	#if MODE_OUTPUT == 0 
		// ----- Output Compare Mode -----
		TIM2 -> CCR1 = 2500;
	
		 // Configure CH1 as output compare
		TIM2 -> CCER &= ~TIM_CCER_CC1E;                         // Disable channel before setup
		TIM2 -> CCMR1 &= ~TIM_CCMR1_CC1S;                       // CC1 channel set as output
		TIM2 -> CCMR1 &= ~TIM_CCMR1_OC1PE;                      // Disable preload
		TIM2 -> CCMR1 &= ~TIM_CCMR1_OC1M;                       // Clear output mode bits
		TIM2 -> CCMR1 |= (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1); // Toggle mode on match
		TIM2 -> CCMR1 &= ~TIM_CCMR1_OC1CE;                      // Disable clear on external event
		TIM2 -> CCER |= TIM_CCER_CC1E;                          // Enable channel output
		
		// // Enable Output Compare interrupt CH1
		TIM2 -> DIER |= TIM_DIER_CC1IE;
		
	#elif MODE_OUTPUT == 1
		// ----- PWM Output Mode (PWM1) -----
		TIM2 -> CCR1 = 1000;
		
		TIM2 -> CCER &= ~TIM_CCER_CC1E;                          // Disable channel before setup
		TIM2 -> CCMR1 &= ~TIM_CCMR1_CC1S;                        // CC1 as output
		TIM2 -> CCMR1 |= TIM_CCMR1_OC1PE;                        // Enable preload, avoid updating register before counting is finished
		TIM2 -> CR1 |= TIM_CR1_ARPE;                             // Enable ARR preload
		TIM2 -> CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);  // PWM mode 1
		TIM2 -> CCER |= TIM_CCER_CC1E;                           // Enable channel output                 
	#endif
	
	// Enable TIM2 counter
	TIM2 -> CR1 |= TIM_CR1_CEN;
}

// ===== TIM3 Configuration =====
// Measures incoming signal using either Input Capture or PWM Input
void Tim3_Input_Config(void)
{
	// CLK for TIM3
	RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3 -> CR1 &= ~TIM_CR1_CEN;
	
	// Count up mode
	TIM3 -> CR1 &= ~ TIM_CR1_DIR;
	TIM3 -> PSC = 719;
	TIM3 -> ARR = 0xFFFF;
	TIM3 -> EGR |= TIM_EGR_UG;
	
	#if MODE_INPUT == 0
		// ----- Input Capture Mode -----
		TIM3 -> CCER &= ~TIM_CCER_CC1E;
		TIM3 -> CCMR1 &= ~TIM_CCMR1_CC1S; 
		TIM3 -> CCMR1 |= TIM_CCMR1_CC1S_0;                         // Map TI1 to CC1                   
		TIM3 -> CCMR1 &= ~TIM_CCMR1_IC1PSC;                        // No prescaler               
		TIM3 -> CCMR1 &= ~TIM_CCMR1_IC1F;                          // No digital filter
		TIM3 -> CCMR1 |= 4 << 12;                                  // fSampling = f/2, n = 6
		TIM3 -> CCER  &= ~TIM_CCER_CC1P;                           // Capture on rising edge
	
		// Enable Interrpu
		TIM3 -> DIER |= TIM_DIER_CC1IE;
		TIM3 -> CCER |= TIM_CCER_CC1E;                             // Enable channel
	#elif MODE_INPUT == 1
		 // ----- PWM Input Mode -----
		TIM3 -> CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);          // Disable channels
		TIM3 -> CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
		TIM3 -> CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1);    // CC1=TI1, CC2=TI1 (indirect)
		TIM3 -> CCMR1 &= ~(TIM_CCMR1_IC1PSC | TIM_CCMR1_IC2PSC);   // No prescaler
		TIM3 -> CCMR1 &= ~(TIM_CCMR1_IC1F | TIM_CCMR1_IC2F);       // No filter
		TIM3 -> CCMR1 |= (4 << 4 | 4 << 12);                       // fSampling = f/2, n = 6
		
		TIM3 -> CCER &= ~TIM_CCER_CC1P;                            // CC1 capture on rising edge
		TIM3 -> CCER |= TIM_CCER_CC2P;                             // CC2 capture on falling edge
		
		// Enable CC1 and CC2 interrupts
		TIM3 -> DIER |= (TIM_DIER_CC1IE | TIM_DIER_CC2IE);
		
		// Configure slave mode controller for reset mode
		TIM3 -> SMCR &= ~TIM_SMCR_SMS;
		TIM3 -> SMCR |= (TIM_SMCR_SMS_2);                          // Reset counter on trigger
		TIM3 -> SMCR &= ~TIM_SMCR_TS;
		TIM3 -> SMCR |= (TIM_SMCR_TS_2 | TIM_SMCR_TS_0);           // Trigger = TI1FP1, CH1
		
		TIM3 -> CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);           // Enable both channels
	#endif
	
	// Enable Tim3
	TIM3 -> CR1 |= TIM_CR1_CEN;
}

// ===== NVIC Configuration =====
// Sets interrupt priority grouping and enables TIM2/TIM3 interrupts
void Cauhinh_NVIC(void)
{
	// Set priority grouping to PRIGROUP7 (0 bits for preemption, 4 bits for subpriority)
	uint32_t temp = SCB -> AIRCR;
	temp |= (0x5FA << 16) | SCB_AIRCR_PRIGROUP7;
	SCB -> AIRCR = temp;
	
	 // Enable TIM2 interrupt, priority = 1
	NVIC -> ISER[TIM2_IRQn >> 5] = 1 << (TIM2_IRQn & 0x1F);
	NVIC -> IP[TIM2_IRQn] = 1 << 4;
	
	// Enable TIM3 interrupt, priority = 2
	NVIC -> ISER[TIM3_IRQn >> 5] = 1 << (TIM3_IRQn & 0x1F);
	NVIC -> IP[TIM3_IRQn] = 2 << 4;
}

// ===== TIM2 Interrupt Handler =====
// Toggles LED when Output Compare event occurs
void TIM2_IRQHandler(void)
{
	if(TIM2 -> SR & TIM_SR_CC1IF)                // Check CC1 interrupt flag
	{
		GPIOA -> ODR ^= 1 << 3;
		TIM2 -> SR &= ~TIM_SR_CC1IF;               // Clear interrupt flag
	}
}

// ===== TIM3 Interrupt Handler =====
// Processes captured input values to calculate period, frequency, and duty cycle
void TIM3_IRQHandler(void)
{
	#if MODE_INPUT == 0
		// ----- Input Capture Mode -----
		if(TIM3 -> SR & TIM_SR_CC1IF)
		{
			uint32_t current_capture_rising = TIM3 -> CCR1;
			TIM3 -> SR &= ~TIM_SR_CC1IF;
			
			if(last_capture_rising != 0)
			{
				if(current_capture_rising >= last_capture_rising)
				{
					period = current_capture_rising - last_capture_rising;
				}
				else period = 0xFFFF + current_capture_rising - last_capture_rising + 1;
				
				frequency = 100000 / period;
			}
			last_capture_rising = current_capture_rising;
		}
	#elif MODE_INPUT == 1
		// ----- PWM Input Mode -----
		if(TIM3 -> SR & TIM_SR_CC1IF)      
		{
			period = TIM3 -> CCR1;                              // Full period in ticks
			
			if(period != 0)
			{
				duty_cycle = ((TIM3 -> CCR2 + 1) * 100) / period;
				frequency = 100000 / period;
			}
			else
			{
				duty_cycle = 0;
				frequency = 0;
			}
		}
	#endif
}
