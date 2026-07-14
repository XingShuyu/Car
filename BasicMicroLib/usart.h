#ifndef	__USART_H__
#define __USART_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdio.h>

typedef void (*USART_RxByteCallback)(uint8_t data);

void USART_Init(void);
void USART_SetRxByteCallback(USART_RxByteCallback callback);

void USART_SendData(UART_Regs *uart, unsigned char data);
void USART_WriteAsync(const char *str);
void USART_PollTx(void);
void USART_IRQHandler(void);
uint32_t USART_GetDroppedTxBytes(void);
uint32_t USART_GetDroppedRxBytes(void);

#endif
