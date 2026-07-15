#ifndef MAIXCAM_PROTOCOL_H
#define MAIXCAM_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#define MAIXCAM_PROTOCOL_ADDR_CAR 0x00
#define MAIXCAM_PROTOCOL_ADDR_EMM 0x01
#define MAIXCAM_PROTOCOL_ADDR_MAIXCAM 0x02

typedef enum MaixCamProtocol_Type {
	MaixCamProtocolType_Beep = 0x00,
	MaixCamProtocolType_Stage = 0x01,
	MaixCamProtocolType_Continue = 0x02,
	MaixCamProtocolType_Stop = 0x03,
	MaixCamProtocolType_Oled = 0x04,
	MaixCamProtocolType_Data = 0x05
} MaixCamProtocol_Type;

typedef struct MaixCamProtocol_Frame {
	uint8_t addr;
	uint8_t type;
	const uint8_t *data;
	uint16_t dataLength;
} MaixCamProtocol_Frame;

static inline bool MaixCamProtocol_ParseFrame(const uint8_t *frame,
											  uint16_t length,
											  MaixCamProtocol_Frame *out)
{
	if (frame == NULL || out == NULL || length < 2U) {
		return false;
	}

	out->addr = frame[0];
	out->type = frame[1];
	out->data = &frame[2];
	out->dataLength = (uint16_t)(length - 2U);
	return true;
}

static inline bool MaixCamProtocol_BuildFrame(uint8_t *buffer,
											  uint16_t bufferSize,
											  uint8_t addr, uint8_t type,
											  const uint8_t *data,
											  uint16_t dataLength,
											  uint16_t *outLength)
{
	uint16_t i;
	uint16_t length = (uint16_t)(2U + dataLength + 2U);

	if (buffer == NULL || bufferSize < length) {
		return false;
	}

	buffer[0] = addr;
	buffer[1] = type;
	for (i = 0; i < dataLength; i++) {
		buffer[2U + i] = data[i];
	}
	buffer[2U + dataLength] = '\r';
	buffer[3U + dataLength] = '\n';

	if (outLength != NULL) {
		*outLength = length;
	}
	return true;
}

static inline bool MaixCamProtocol_ParseStageData(const uint8_t *data,
												  uint16_t dataLength,
												  int *stageIndex)
{
	char text[12];
	char *end = NULL;
	uint16_t copyLength;
	long parsed;

	if (data == NULL || dataLength == 0U || stageIndex == NULL) {
		return false;
	}

	if (dataLength == 1U) {
		*stageIndex = data[0];
		return true;
	}

	copyLength = dataLength;
	if (copyLength >= sizeof(text)) {
		copyLength = (uint16_t)(sizeof(text) - 1U);
	}
	for (uint16_t i = 0; i < copyLength; i++) {
		text[i] = (char)data[i];
	}
	text[copyLength] = '\0';

	parsed = strtol(text, &end, 0);
	if (end == text || *end != '\0' || parsed < 0) {
		return false;
	}

	*stageIndex = (int)parsed;
	return true;
}

static inline bool MaixCamProtocol_ParseOledData(const uint8_t *data,
												 uint16_t dataLength,
												 uint8_t *row,
												 uint8_t *col,
												 const uint8_t **text,
												 uint16_t *textLength)
{
	if (data == NULL || dataLength == 0U || row == NULL || col == NULL ||
		text == NULL || textLength == NULL) {
		return false;
	}

	*row = 0U;
	*col = 0U;
	*text = data;
	*textLength = dataLength;

	if (dataLength >= 3U && data[0] <= 7U && data[1] <= 20U) {
		*row = data[0];
		*col = data[1];
		*text = &data[2];
		*textLength = (uint16_t)(dataLength - 2U);
	}

	return *textLength > 0U;
}

#endif
