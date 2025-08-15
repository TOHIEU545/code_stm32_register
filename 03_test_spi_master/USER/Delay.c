#include "Delay.h"

void Delay_Sys(uint16_t ms){
	SysTick->LOAD = 72000u - 1u;
	SysTick->VAL = 0u;
	SysTick->CTRL = 1u | 1u << 2;
	while(ms--){
	while((SysTick->CTRL & 1<<16) == 0);
	}
	SysTick -> CTRL = 0;
}
