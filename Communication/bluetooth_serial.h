#ifndef BLUETOOTH_SERIAL_H
#define BLUETOOTH_SERIAL_H

#include <stdint.h>

const volatile uint8_t *BluetoothSerial_GetBuffer(void);
uint16_t BluetoothSerial_GetLength(void);
uint8_t BluetoothSerial_GetFlag(void);
void BluetoothSerial_ClearFlag(void);
void BluetoothSerial_Init(void);

#endif
