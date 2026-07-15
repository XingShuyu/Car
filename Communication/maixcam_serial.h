#ifndef MAIXCAM_SERIAL_H
#define MAIXCAM_SERIAL_H

#include <stdbool.h>
#include <stdint.h>

const volatile uint8_t *MaixCamSerial_GetBuffer(void);
uint16_t MaixCamSerial_GetLength(void);
uint8_t MaixCamSerial_GetFlag(void);
void MaixCamSerial_ClearFlag(void);
bool MaixCamSerial_TryReadFrame(uint8_t *buffer, uint16_t bufferSize,
								uint16_t *length);
void MaixCamSerial_SendBytes(const uint8_t *data, uint16_t length);
void MaixCamSerial_Init(void);
void MaixCamSerial_Poll(void);
void MaixCamSerial_IRQHandler(void);

#endif
