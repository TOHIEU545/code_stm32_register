#include "Debug.h"

//struct __FILE {
//    int handle;
//};

FILE __stdout;
int fputc(int chr, FILE *f){
	SendChar((char)chr);
	return chr;
}

void Uart_Config(void){
	USART_InitTypeDef uart;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	uart.USART_BaudRate = 9600;
	uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	uart.USART_Parity = USART_Parity_No;
	uart.USART_StopBits = USART_StopBits_1;
	uart.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &uart);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
}

void SendChar(char chr){
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1, chr);
}

void SendString(char* str){
	while(*str != NULL){
		SendChar(*str++);
	}
}
