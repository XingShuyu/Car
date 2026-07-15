#include "usart.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#define USART_TX_BUFFER_SIZE 1024U
#ifndef DMA_UART0_TX_CHAN_ID
#define DMA_UART0_TX_CHAN_ID 0U
#endif
#ifndef DMA_UART0_RX_CHAN_ID
#define DMA_UART0_RX_CHAN_ID 1U
#endif
#define USART_DMA_TX_CHAN_ID DMA_UART0_TX_CHAN_ID
#define USART_DMA_RX_CHAN_ID DMA_UART0_RX_CHAN_ID
#define USART_DMA_RX_BLOCK_SIZE 64U
#define USART_RX_FIFO_DRAIN_LIMIT 32U

static volatile uint8_t usartTxBuffer[USART_TX_BUFFER_SIZE];
static volatile uint16_t usartTxHead = 0;
static volatile uint16_t usartTxTail = 0;
static volatile uint32_t usartTxDropped = 0;
static volatile bool usartTxDmaBusy = false;
static volatile uint16_t usartTxDmaLen = 0;

static volatile uint8_t usartRxDmaBlock[USART_DMA_RX_BLOCK_SIZE];
static volatile uint16_t usartRxDmaProcessed = 0;
static volatile uint32_t usartRxDropped = 0;
static USART_RxByteCallback usartRxCallback = NULL;

static void USART_RestoreIrq(uint32_t primask)
{
	if (primask == 0U) {
		__enable_irq();
	}
}

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

static void USART_FlushRxFifo(void)
{
	uint32_t drained = 0;

	while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST) &&
		   (drained < USART_RX_FIFO_DRAIN_LIMIT)) {
		(void)DL_UART_Main_receiveData(UART_0_INST);
		drained++;
	}
}

static void USART_StartRxDma(void)
{
	DL_DMA_disableChannel(DMA, USART_DMA_RX_CHAN_ID);
	usartRxDmaProcessed = 0;
	DL_DMA_setSrcAddr(DMA, USART_DMA_RX_CHAN_ID,
					  (uint32_t)&UART_0_INST->RXDATA);
	DL_DMA_setDestAddr(DMA, USART_DMA_RX_CHAN_ID,
					   (uint32_t)&usartRxDmaBlock[0]);
	DL_DMA_setTransferSize(DMA, USART_DMA_RX_CHAN_ID,
						   USART_DMA_RX_BLOCK_SIZE);
	DL_UART_Main_clearInterruptStatus(UART_0_INST,
									  DL_UART_MAIN_INTERRUPT_DMA_DONE_RX |
										  DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
	DL_DMA_enableChannel(DMA, USART_DMA_RX_CHAN_ID);
}

static void USART_HandleRxByte(uint8_t data)
{
	USART_RxByteCallback callback = usartRxCallback;

	if (callback != NULL) {
		callback(data);
	} else {
		usartRxDropped++;
	}
}

static uint16_t USART_GetRxDmaReceived(void)
{
	uint16_t remaining = DL_DMA_getTransferSize(DMA, USART_DMA_RX_CHAN_ID);

	if (remaining > USART_DMA_RX_BLOCK_SIZE) {
		remaining = 0;
	}
	return (uint16_t)(USART_DMA_RX_BLOCK_SIZE - remaining);
}

static void USART_ProcessRxDmaAvailable(bool restartWhenFull)
{
	uint16_t received = USART_GetRxDmaReceived();

	if (received > USART_DMA_RX_BLOCK_SIZE) {
		received = USART_DMA_RX_BLOCK_SIZE;
	}

	while (usartRxDmaProcessed < received) {
		USART_HandleRxByte((uint8_t)usartRxDmaBlock[usartRxDmaProcessed]);
		usartRxDmaProcessed++;
	}

	if (restartWhenFull && (received >= USART_DMA_RX_BLOCK_SIZE)) {
		USART_StartRxDma();
	}
}

static void USART_PollRxDma(void)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	USART_ProcessRxDmaAvailable(true);
	USART_RestoreIrq(primask);
}

static void USART_DrainRxFifoToCallback(void)
{
	uint32_t drained = 0;

	while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST) &&
		   (drained < USART_RX_FIFO_DRAIN_LIMIT)) {
		USART_HandleRxByte(DL_UART_Main_receiveData(UART_0_INST));
		drained++;
	}
}

static void USART_StartTxDmaLocked(void)
{
	uint16_t tail;
	uint16_t head;
	uint16_t len;

	if (usartTxDmaBusy || (usartTxHead == usartTxTail)) {
		return;
	}

	tail = usartTxTail;
	head = usartTxHead;
	if (head > tail) {
		len = (uint16_t)(head - tail);
	} else {
		len = (uint16_t)(USART_TX_BUFFER_SIZE - tail);
	}

	if (len == 0U) {
		return;
	}

	usartTxDmaBusy = true;
	usartTxDmaLen = len;

	DL_DMA_disableChannel(DMA, USART_DMA_TX_CHAN_ID);
	DL_DMA_setSrcAddr(DMA, USART_DMA_TX_CHAN_ID,
					  (uint32_t)&usartTxBuffer[tail]);
	DL_DMA_setDestAddr(DMA, USART_DMA_TX_CHAN_ID,
					   (uint32_t)&UART_0_INST->TXDATA);
	DL_DMA_setTransferSize(DMA, USART_DMA_TX_CHAN_ID, len);
	DL_UART_Main_clearInterruptStatus(UART_0_INST,
									  DL_UART_MAIN_INTERRUPT_DMA_DONE_TX |
										  DL_UART_MAIN_INTERRUPT_EOT_DONE);
	DL_DMA_enableChannel(DMA, USART_DMA_TX_CHAN_ID);
}

static void USART_KickTxDma(void)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	USART_StartTxDmaLocked();
	USART_RestoreIrq(primask);
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
	USART_RestoreIrq(primask);

	return queued;
}

static void USART_OnTxDmaDone(void)
{
	DL_DMA_disableChannel(DMA, USART_DMA_TX_CHAN_ID);

	if (usartTxDmaBusy) {
		usartTxTail = (uint16_t)((usartTxTail + usartTxDmaLen) %
								 USART_TX_BUFFER_SIZE);
	}
	usartTxDmaLen = 0;
	usartTxDmaBusy = false;

	USART_StartTxDmaLocked();
}

void USART_Init(void)
{
	usartTxHead = 0;
	usartTxTail = 0;
	usartTxDropped = 0;
	usartTxDmaBusy = false;
	usartTxDmaLen = 0;
	usartRxDropped = 0;
	usartRxDmaProcessed = 0;

	// SysConfig 已完成 UART0/DMA 通道配置，这里只设置运行期缓冲。
	DL_UART_Main_enableFIFOs(UART_0_INST);
	DL_UART_Main_setTXFIFOThreshold(UART_0_INST,
									DL_UART_MAIN_TX_FIFO_LEVEL_ONE_ENTRY);
	DL_UART_Main_setRXFIFOThreshold(UART_0_INST,
									DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
	DL_UART_Main_setRXInterruptTimeout(UART_0_INST, 15);
	DL_UART_Main_disableInterrupt(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX);
	DL_UART_Main_enableDMAReceiveEvent(UART_0_INST,
									   DL_UART_MAIN_DMA_INTERRUPT_RX);
	DL_UART_Main_enableDMATransmitEvent(UART_0_INST);
	DL_UART_Main_enableInterrupt(UART_0_INST,
								 DL_UART_MAIN_INTERRUPT_DMA_DONE_RX |
									 DL_UART_MAIN_INTERRUPT_DMA_DONE_TX |
									 DL_UART_MAIN_INTERRUPT_EOT_DONE |
									 DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
									 DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
									 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR);
	DL_UART_Main_clearInterruptStatus(UART_0_INST,
									  DL_UART_MAIN_INTERRUPT_RX |
										  DL_UART_MAIN_INTERRUPT_DMA_DONE_RX |
										  DL_UART_MAIN_INTERRUPT_DMA_DONE_TX |
										  DL_UART_MAIN_INTERRUPT_EOT_DONE |
										  DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
										  DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
										  DL_UART_MAIN_INTERRUPT_FRAMING_ERROR);
	USART_FlushRxFifo();
	USART_StartRxDma();
	DL_UART_Main_enable(UART_0_INST);

	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void USART_SetRxByteCallback(USART_RxByteCallback callback)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	usartRxCallback = callback;
	USART_RestoreIrq(primask);
}

void USART_SendData(UART_Regs *uart, unsigned char data)
{
	if (uart == UART_0_INST) {
		if (USART_EnqueueByte((uint8_t)data)) {
			USART_KickTxDma();
		}
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
	USART_KickTxDma();
}

void USART_PollTx(void)
{
	USART_PollRxDma();
	USART_KickTxDma();
}

void USART_IRQHandler(void)
{
	DL_UART_IIDX pending;

	do {
		pending = DL_UART_Main_getPendingInterrupt(UART_0_INST);
		switch (pending) {
		case DL_UART_MAIN_IIDX_DMA_DONE_RX:
			USART_ProcessRxDmaAvailable(true);
			break;
		case DL_UART_MAIN_IIDX_DMA_DONE_TX:
			USART_OnTxDmaDone();
			break;
		case DL_UART_MAIN_IIDX_EOT_DONE:
			break;
		case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
			USART_ProcessRxDmaAvailable(false);
			break;
		case DL_UART_MAIN_IIDX_RX:
			USART_DrainRxFifoToCallback();
			break;
		case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
		case DL_UART_MAIN_IIDX_FRAMING_ERROR:
			usartRxDropped++;
			USART_FlushRxFifo();
			USART_StartRxDma();
			break;
		default:
			break;
		}
	} while (pending != DL_UART_MAIN_IIDX_NO_INTERRUPT);
}

uint32_t USART_GetDroppedTxBytes(void)
{
	return usartTxDropped;
}

uint32_t USART_GetDroppedRxBytes(void)
{
	return usartRxDropped;
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
	USART_KickTxDma();
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


