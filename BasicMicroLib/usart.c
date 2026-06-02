#include "usart.h"
#include <stdbool.h>
#include <stdio.h>


#define RE_0_BUFF_LEN_MAX	128
#define USART_TX_BUFFER_SIZE 1024U
#define USART_TX_BUDGET_PER_POLL 16U

static volatile uint8_t usartTxBuffer[USART_TX_BUFFER_SIZE];
static volatile uint16_t usartTxHead = 0;
static volatile uint16_t usartTxTail = 0;
static volatile uint32_t usartTxDropped = 0;

static void USART_SendByte_Blocking(UART_Regs *uart, uint8_t data)
{
	uint32_t timeout = 1000000U;
	while ((DL_UART_isBusy(uart) == true) && (timeout > 0U)) {
		timeout--;
	}
	if (timeout == 0U) {
		return;
	}
	DL_UART_Main_transmitData(uart, data);
}

static bool USART_EnqueueByte(uint8_t data)
{
	uint32_t primask;
	uint16_t head;
	uint16_t nextHead;
	bool queued = false;

	primask = __get_PRIMASK();
	__disable_irq();
	head = usartTxHead;
	nextHead = (uint16_t)((head + 1U) % USART_TX_BUFFER_SIZE);
	if (nextHead != usartTxTail) {
		usartTxBuffer[head] = data;
		usartTxHead = nextHead;
		queued = true;
	} else {
		usartTxDropped++;
	}
	if (primask == 0U) {
		__enable_irq();
	}

	return queued;
}

void USART_Init(void)
{
	usartTxHead = 0;
	usartTxTail = 0;
	usartTxDropped = 0;

	DL_UART_Main_changeConfig(UART_0_INST);
	DL_UART_Main_enableFIFOs(UART_0_INST);
	DL_UART_Main_setTXFIFOThreshold(UART_0_INST,
									DL_UART_MAIN_TX_FIFO_LEVEL_EMPTY);
	DL_UART_Main_setRXFIFOThreshold(UART_0_INST,
									DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
	DL_UART_Main_enable(UART_0_INST);
	//清除串口中断标志
	//Clear the serial port interrupt flag
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	//使能串口中断
	//Enable serial port interrupt
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void USART_SendData(UART_Regs *uart, unsigned char data)
{
	if (uart == UART_0_INST) {
		(void)USART_EnqueueByte((uint8_t)data);
	} else {
		USART_SendByte_Blocking(uart, (uint8_t)data);
	}
}

void USART_WriteAsync(const char *str)
{
	if (str == NULL) {
		return;
	}

	while (*str != '\0') {
		(void)USART_EnqueueByte((uint8_t)*str);
		str++;
	}
}

void USART_PollTx(void)
{
	uint32_t sent = 0;

	while ((sent < USART_TX_BUDGET_PER_POLL) && (usartTxTail != usartTxHead) &&
		   !DL_UART_Main_isTXFIFOFull(UART_0_INST)) {
		DL_UART_Main_transmitData(UART_0_INST, usartTxBuffer[usartTxTail]);
		usartTxTail = (uint16_t)((usartTxTail + 1U) % USART_TX_BUFFER_SIZE);
		sent++;
	}
}

uint32_t USART_GetDroppedTxBytes(void)
{
	return usartTxDropped;
}


#if !defined(__MICROLIB)
//不使用微库的话就需要添加下面的函数
//If you don't use the micro library, you need to add the following function
#if (__ARMCLIB_VERSION <= 6000000)
//如果编译器是AC5  就定义下面这个结构体
//If the compiler is AC5, define the following structure
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

//定义_sys_exit()以避免使用半主机模式
//Define _sys_exit() to avoid using semihosting mode
void _sys_exit(int x)
{
    (void)x;
    while (1)
    {
        __WFI();  // 等待中断，降低功耗
    }
}
#endif

// tiarmclang 下重定向 printf，补齐常见输出入口，避免落到 CIO

int fputc(int ch, FILE *stream)
{
	(void)stream;
	if (ch == '\n') {
		(void)USART_EnqueueByte('\r');
	}
	(void)USART_EnqueueByte((uint8_t)ch);
	return ch;
}

int putchar(int ch)
{
	return fputc(ch, stdout);
}

int write(int fd, const char *buf, unsigned int count)
{
	unsigned int i;
	(void)fd;
	if (buf == NULL) {
		return 0;
	}
	for (i = 0; i < count; i++) {
		fputc((unsigned char)buf[i], stdout);
	}
	return (int)count;
}


