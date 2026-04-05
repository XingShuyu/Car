#include "usart.h"
#include <stdio.h>


#define RE_0_BUFF_LEN_MAX	128



static void USART_SendByte_Blocking(uint8_t data)
{
	uint32_t timeout = 1000000U;
	while ((DL_UART_isBusy(UART_0_INST) == true) && (timeout > 0U)) {
		timeout--;
	}
	if (timeout == 0U) {
		return;
	}
	DL_UART_Main_transmitData(UART_0_INST, data);
}

void USART_Init(void)
{
	//清除串口中断标志
	//Clear the serial port interrupt flag
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	//使能串口中断
	//Enable serial port interrupt
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

//串口发送一个字节
//The serial port sends a byte
void USART_SendData(unsigned char data)
{
	USART_SendByte_Blocking((uint8_t)data);
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
		USART_SendByte_Blocking('\r');
	}
	USART_SendByte_Blocking((uint8_t)ch);
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


