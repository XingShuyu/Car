#include "Communication/bluetooth_serial.h"

#include "BasicMicroLib/usart.h"
#include <stdbool.h>

#define BLUETOOTH_SERIAL_BUFFER_SIZE 128U

// UART1 蓝牙接收帧缓存，recv0_buff 保存最近一帧完整 CRLF 数据。
static volatile uint8_t recv0_buff[BLUETOOTH_SERIAL_BUFFER_SIZE] = {0};
static volatile uint16_t recv0_length = 0;
static volatile uint8_t recv0_flag = 0;
static uint8_t recv0_work[BLUETOOTH_SERIAL_BUFFER_SIZE] = {0};
static uint16_t recv0_work_length = 0;
static bool recv0_seen_cr = false;
static bool recv0_overflow = false;

static void BluetoothSerial_ResetWork(void)
{
	recv0_work_length = 0;
	recv0_work[0] = '\0';
	recv0_seen_cr = false;
	recv0_overflow = false;
}

static void BluetoothSerial_AppendWork(uint8_t data)
{
	if (recv0_work_length < (BLUETOOTH_SERIAL_BUFFER_SIZE - 1U)) {
		recv0_work[recv0_work_length++] = data;
	} else {
		BluetoothSerial_ResetWork();
		recv0_overflow = true;
	}
}

static void BluetoothSerial_PublishFrame(void)
{
	uint16_t i;

	for (i = 0; i < recv0_work_length; i++) {
		recv0_buff[i] = recv0_work[i];
	}
	recv0_buff[recv0_work_length] = '\0';
	recv0_length = recv0_work_length;
	recv0_flag = 1;
	BluetoothSerial_ResetWork();
}

static void BluetoothSerial_OnRxByte(uint8_t receivedData)
{
	if (recv0_overflow) {
		if (recv0_seen_cr && (receivedData == '\n')) {
			BluetoothSerial_ResetWork();
		} else {
			recv0_seen_cr = (receivedData == '\r');
		}
		return;
	}

	if (recv0_seen_cr) {
		if (receivedData == '\n') {
			BluetoothSerial_PublishFrame();
			return;
		}
		BluetoothSerial_AppendWork('\r');
		recv0_seen_cr = false;
		if (recv0_overflow) {
			return;
		}
	}

	if (receivedData == '\r') {
		recv0_seen_cr = true;
	} else if (receivedData != '\n') {
		BluetoothSerial_AppendWork(receivedData);
	}
}

const volatile uint8_t *BluetoothSerial_GetBuffer(void)
{
	return recv0_buff;
}

uint16_t BluetoothSerial_GetLength(void)
{
	return recv0_length;
}

uint8_t BluetoothSerial_GetFlag(void)
{
	return recv0_flag;
}

void BluetoothSerial_ClearFlag(void)
{
	recv0_flag = 0;
}

void BluetoothSerial_Init(void)
{
	recv0_length = 0;
	recv0_flag = 0;
	recv0_buff[0] = '\0';
	BluetoothSerial_ResetWork();
	USART_SetRxByteCallback(BluetoothSerial_OnRxByte);
}
