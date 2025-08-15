#include "stm32f10x.h"
#include "Delay.h"
#include "stm32f10x_i2c.h"
#include "Lcd_i2c.h"

int main(){
	LCD_I2C_Init();
	LCD_Begin(2);
	LCD_SetEntryMode(1, 0);
	LCD_SendData('A');
	while(1){
		
	}
}
