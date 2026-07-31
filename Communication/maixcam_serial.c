#include "Communication/maixcam_serial.h"

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stddef.h>

#ifndef DMA_MAIXCAM_RX_CHAN_ID
#define DMA_MAIXCAM_RX_CHAN_ID 2U
#endif

#define MAIXCAM_SERIAL_BUFFER_SIZE 32U
#define MAIXCAM_DMA_RX_BLOCK_SIZE 32U
#define MAIXCAM_RX_FIFO_DRAIN_LIMIT 32U
#define MAIXCAM_SERIAL_USE_DMA_RX 0
#define MAIXCAM_SERIAL_ARRIVED_TEXT "Arrived"
#define MAIXCAM_SERIAL_ARRIVED_LENGTH \
	((uint16_t)(sizeof(MAIXCAM_SERIAL_ARRIVED_TEXT) - 1U))

// MaixCam 串口接收帧缓存，maixcam_buff 保存最近一帧完整 CRLF 数据。
static volatile uint8_t maixcam_buff[MAIXCAM_SERIAL_BUFFER_SIZE] = {0};
static volatile uint16_t maixcam_length = 0;
static volatile uint8_t maixcam_flag = 0;
static volatile uint8_t maixcam_arrived_event = 0;
#if MAIXCAM_SERIAL_USE_DMA_RX
static volatile uint8_t maixcam_rx_dma_block[MAIXCAM_DMA_RX_BLOCK_SIZE] = {0};
static volatile uint16_t maixcam_rx_dma_processed = 0;
#endif
static uint8_t maixcam_work[MAIXCAM_SERIAL_BUFFER_SIZE] = {0};
static uint16_t maixcam_work_length = 0;
static bool maixcam_seen_cr = false;
static bool maixcam_overflow = false;

static void MaixCamSerial_SendByte(uint8_t data)
{
	DL_UART_Main_transmitDataBlocking(UART_MAIXCAM_INST, data);
}

static void MaixCamSerial_ResetWork(void)
{
	maixcam_work_length = 0;
	maixcam_work[0] = '\0';
	maixcam_seen_cr = false;
	maixcam_overflow = false;
}

static void MaixCamSerial_AppendWork(uint8_t data)
{
	if (maixcam_work_length < (MAIXCAM_SERIAL_BUFFER_SIZE - 1U)) {
		maixcam_work[maixcam_work_length++] = data;
	} else {
		MaixCamSerial_ResetWork();
		maixcam_overflow = true;
	}
}

static bool MaixCamSerial_IsArrivedFrame(void)
{
	uint16_t i;

	if (maixcam_work_length != MAIXCAM_SERIAL_ARRIVED_LENGTH) {
		return false;
	}

	for (i = 0U; i < MAIXCAM_SERIAL_ARRIVED_LENGTH; i++) {
		if (maixcam_work[i] !=
			(uint8_t)MAIXCAM_SERIAL_ARRIVED_TEXT[i]) {
			return false;
		}
	}
	return true;
}

static void MaixCamSerial_PublishFrame(void)
{
	uint16_t i;

	for (i = 0; i < maixcam_work_length; i++) {
		maixcam_buff[i] = maixcam_work[i];
	}
	maixcam_buff[maixcam_work_length] = '\0';
	maixcam_length = maixcam_work_length;
	maixcam_flag = 1;
	if (MaixCamSerial_IsArrivedFrame()) {
		maixcam_arrived_event = 1U;
	}
	MaixCamSerial_ResetWork();
}

static void MaixCamSerial_ProcessByte(uint8_t data)
{
	if (maixcam_overflow) {
		if (maixcam_seen_cr && (data == '\n')) {
			MaixCamSerial_ResetWork();
		} else {
			maixcam_seen_cr = (data == '\r');
		}
		return;
	}

	if (maixcam_seen_cr) {
		if (data == '\n') {
			MaixCamSerial_PublishFrame();
			return;
		}
		MaixCamSerial_AppendWork('\r');
		maixcam_seen_cr = false;
		if (maixcam_overflow) {
			return;
		}
	}

	if (data == '\r') {
		maixcam_seen_cr = true;
	} else if (data != '\n') {
		MaixCamSerial_AppendWork(data);
	}
}

static void MaixCamSerial_FlushRxFifo(void)
{
	uint32_t drained = 0;

	while (!DL_UART_Main_isRXFIFOEmpty(UART_MAIXCAM_INST) &&
		   (drained < MAIXCAM_RX_FIFO_DRAIN_LIMIT)) {
		(void)DL_UART_Main_receiveData(UART_MAIXCAM_INST);
		drained++;
	}
}

static void MaixCamSerial_DrainRxFifo(void)
{
	uint32_t drained = 0;

	while (!DL_UART_Main_isRXFIFOEmpty(UART_MAIXCAM_INST) &&
		   (drained < MAIXCAM_RX_FIFO_DRAIN_LIMIT)) {
		MaixCamSerial_ProcessByte(
			DL_UART_Main_receiveData(UART_MAIXCAM_INST));
		drained++;
	}
}

#if MAIXCAM_SERIAL_USE_DMA_RX
static void MaixCamSerial_StartRxDma(void)
{
	DL_DMA_disableChannel(DMA, DMA_MAIXCAM_RX_CHAN_ID);
	maixcam_rx_dma_processed = 0;
	DL_DMA_setSrcAddr(DMA, DMA_MAIXCAM_RX_CHAN_ID,
					  (uint32_t)&UART_MAIXCAM_INST->RXDATA);
	DL_DMA_setDestAddr(DMA, DMA_MAIXCAM_RX_CHAN_ID,
					   (uint32_t)&maixcam_rx_dma_block[0]);
	DL_DMA_setTransferSize(DMA, DMA_MAIXCAM_RX_CHAN_ID,
						   MAIXCAM_DMA_RX_BLOCK_SIZE);
	DL_UART_Main_clearInterruptStatus(UART_MAIXCAM_INST,
									  DL_UART_MAIN_INTERRUPT_DMA_DONE_RX |
										  DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
	DL_DMA_enableChannel(DMA, DMA_MAIXCAM_RX_CHAN_ID);
}

static uint16_t MaixCamSerial_GetRxDmaReceived(void)
{
	uint16_t remaining =
		DL_DMA_getTransferSize(DMA, DMA_MAIXCAM_RX_CHAN_ID);

	if (remaining > MAIXCAM_DMA_RX_BLOCK_SIZE) {
		remaining = 0;
	}
	return (uint16_t)(MAIXCAM_DMA_RX_BLOCK_SIZE - remaining);
}

static void MaixCamSerial_ProcessRxDmaAvailable(bool restartWhenFull)
{
	uint16_t received = MaixCamSerial_GetRxDmaReceived();

	if (received > MAIXCAM_DMA_RX_BLOCK_SIZE) {
		received = MAIXCAM_DMA_RX_BLOCK_SIZE;
	}

	while (maixcam_rx_dma_processed < received) {
		MaixCamSerial_ProcessByte(
			(uint8_t)maixcam_rx_dma_block[maixcam_rx_dma_processed]);
		maixcam_rx_dma_processed++;
	}

	if (restartWhenFull && (received >= MAIXCAM_DMA_RX_BLOCK_SIZE)) {
		MaixCamSerial_StartRxDma();
	}
}
#endif

const volatile uint8_t *MaixCamSerial_GetBuffer(void)
{
	return maixcam_buff;
}

uint16_t MaixCamSerial_GetLength(void)
{
	return maixcam_length;
}

uint8_t MaixCamSerial_GetFlag(void)
{
	return maixcam_flag;
}

void MaixCamSerial_ClearFlag(void)
{
	maixcam_flag = 0;
}

bool MaixCamSerial_TakeArrivedEvent(void)
{
	bool hasEvent;
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	hasEvent = (maixcam_arrived_event != 0U);
	maixcam_arrived_event = 0U;
	if (primask == 0U) {
		__enable_irq();
	}
	return hasEvent;
}

bool MaixCamSerial_TryReadFrame(uint8_t *buffer, uint16_t bufferSize,
								uint16_t *length)
{
	uint16_t copyLength;
	uint16_t i;
	uint32_t primask;

	if (buffer == NULL || bufferSize == 0U) {
		return false;
	}

	primask = __get_PRIMASK();
	__disable_irq();

	if (maixcam_flag == 0U) {
		if (primask == 0U) {
			__enable_irq();
		}
		return false;
	}

	copyLength = maixcam_length;
	if (copyLength > bufferSize) {
		copyLength = bufferSize;
	}

	for (i = 0; i < copyLength; i++) {
		buffer[i] = maixcam_buff[i];
	}
	maixcam_flag = 0U;

	if (primask == 0U) {
		__enable_irq();
	}

	if (length != NULL) {
		*length = copyLength;
	}
	return true;
}

void MaixCamSerial_SendBytes(const uint8_t *data, uint16_t length)
{
	uint16_t i;

	if (data == NULL) {
		return;
	}

	for (i = 0; i < length; i++) {
		MaixCamSerial_SendByte(data[i]);
	}
}

void MaixCamSerial_Init(void)
{
	maixcam_length = 0;
	maixcam_flag = 0;
	maixcam_arrived_event = 0;
	maixcam_buff[0] = '\0';
#if MAIXCAM_SERIAL_USE_DMA_RX
	maixcam_rx_dma_processed = 0;
#endif
	MaixCamSerial_ResetWork();

	DL_UART_Main_enableFIFOs(UART_MAIXCAM_INST);
	DL_UART_Main_setRXFIFOThreshold(UART_MAIXCAM_INST,
									DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
	DL_UART_Main_setRXInterruptTimeout(UART_MAIXCAM_INST, 15);
	DL_DMA_disableChannel(DMA, DMA_MAIXCAM_RX_CHAN_ID);
#if MAIXCAM_SERIAL_USE_DMA_RX
	DL_UART_Main_disableInterrupt(UART_MAIXCAM_INST, DL_UART_MAIN_INTERRUPT_RX);
	DL_UART_Main_enableDMAReceiveEvent(UART_MAIXCAM_INST,
									   DL_UART_MAIN_DMA_INTERRUPT_RX);
	DL_UART_Main_enableInterrupt(UART_MAIXCAM_INST,
								 DL_UART_MAIN_INTERRUPT_DMA_DONE_RX |
									 DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
									 DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
									 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR);
#else
	DL_UART_Main_enableInterrupt(UART_MAIXCAM_INST,
								 DL_UART_MAIN_INTERRUPT_RX |
									 DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
									 DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
									 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR);
#endif
	DL_UART_Main_clearInterruptStatus(UART_MAIXCAM_INST,
									  DL_UART_MAIN_INTERRUPT_RX |
										  DL_UART_MAIN_INTERRUPT_DMA_DONE_RX |
										  DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
										  DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
										  DL_UART_MAIN_INTERRUPT_FRAMING_ERROR);
	MaixCamSerial_FlushRxFifo();
#if MAIXCAM_SERIAL_USE_DMA_RX
	MaixCamSerial_StartRxDma();
#endif
	DL_UART_Main_enable(UART_MAIXCAM_INST);
}

void MaixCamSerial_Poll(void)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
#if MAIXCAM_SERIAL_USE_DMA_RX
	MaixCamSerial_ProcessRxDmaAvailable(true);
#else
	MaixCamSerial_DrainRxFifo();
#endif
	if (primask == 0U) {
		__enable_irq();
	}
}

void MaixCamSerial_IRQHandler(void)
{
	DL_UART_IIDX pending;

	do {
		pending = DL_UART_Main_getPendingInterrupt(UART_MAIXCAM_INST);
		switch (pending) {
		case DL_UART_MAIN_IIDX_DMA_DONE_RX:
#if MAIXCAM_SERIAL_USE_DMA_RX
			MaixCamSerial_ProcessRxDmaAvailable(true);
#endif
			break;
		case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
#if MAIXCAM_SERIAL_USE_DMA_RX
			MaixCamSerial_ProcessRxDmaAvailable(false);
#else
			MaixCamSerial_DrainRxFifo();
#endif
			break;
		case DL_UART_MAIN_IIDX_RX:
			MaixCamSerial_DrainRxFifo();
			break;
		case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
		case DL_UART_MAIN_IIDX_FRAMING_ERROR:
			MaixCamSerial_FlushRxFifo();
#if MAIXCAM_SERIAL_USE_DMA_RX
			MaixCamSerial_StartRxDma();
#endif
			break;
		default:
			break;
		}
	} while (pending != DL_UART_MAIN_IIDX_NO_INTERRUPT);
}
