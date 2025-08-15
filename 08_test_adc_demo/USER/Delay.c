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

void Delay_Sys_us(uint16_t us){
	SysTick->LOAD = 72u - 1u;
	SysTick->VAL = 0u;
	SysTick->CTRL = 1u | 1u << 2;
	while(us--){
	while((SysTick->CTRL & 1<<16) == 0);
	}
	SysTick -> CTRL = 0;
}
