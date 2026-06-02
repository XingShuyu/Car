#ifndef	__USART_H__
#define __USART_H__

#include "ti_msp_dl_config.h"
#include "stdio.h"

void USART_Init(void);

void USART_SendData(UART_Regs *uart, unsigned char data);
void USART_WriteAsync(const char *str);
void USART_PollTx(void);
uint32_t USART_GetDroppedTxBytes(void);

#endif
