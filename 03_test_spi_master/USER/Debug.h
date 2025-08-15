#ifndef DEBUG_H
#define DEBUG_H
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "string.h"
#include "stdio.h"

void Uart_Config(void);
void SendChar(char chr);
void SendString(char* str);

#endif