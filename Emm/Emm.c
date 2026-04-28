#include "Emm.h"
#include "../BasicMicroLib/usart.h"
#include <stdio.h>
#include <string.h>

void Emm_SendData(Emm_Data *emm_Data) {
    delay_ms(1);
	USART_SendData(Emm_INST, (char)(emm_Data->addr));
	USART_SendData(Emm_INST, (char)(emm_Data->funcCode));
	for (int i = 0; i < emm_Data->dataSize; i++) {
		USART_SendData(Emm_INST, (char)emm_Data->data[i]);
	}
	USART_SendData(Emm_INST, (char)(emm_Data->checkSum));
    
}

void Emm_Loc_Control(int Addr, LocControl *locControl) {
	Emm_Data emm_Data;
	unsigned char temp[10];
	char tmp[4];
	int i;

	// 角度: 度 -> 电机单位 (3200单位 = 360°)
	int angleNum = (int)((locControl->angle) / 360.0f * 3200.0f);
	// 速度: °/s -> RPM (1 RPM = 6°/s)
	int speedRPM = locControl->speed / 6;

	// Byte 0: 旋转方向
	temp[0] = (unsigned char)locControl->dirction;

	// Bytes 1-2: 速度 RPM, 大端序
	memcpy(tmp, &speedRPM, 2);
	temp[1] = tmp[1];
	temp[2] = tmp[0];

	// Byte 3: 加速度挡位
	temp[3] = (unsigned char)locControl->accu;

	// Bytes 4-7: 角度, 大端序
	memcpy(tmp, &angleNum, 4);
	for (i = 0; i < 4; i++)
		temp[4 + i] = tmp[3 - i];

	// Byte 8: 相对/绝对模式
	temp[8] = (unsigned char)locControl->mode;

	// Byte 9: 恒定为 0x00
	temp[9] = 0x00;

	emm_Data.data = temp;
	emm_Data.addr = Addr;
	emm_Data.funcCode = 0xFD;
	emm_Data.dataSize = 10;
	emm_Data.checkSum = 0x6B;
	Emm_SendData(&emm_Data);
}

void Emm_Stop(int Addr) {
	Emm_Data emm_Data;
	emm_Data.addr = Addr;
	emm_Data.funcCode = 0xFE;
	emm_Data.dataSize = 2;
	unsigned char temp[] = {0x98, 0x00};
	emm_Data.data = temp;
	emm_Data.checkSum = 0x6B;
	Emm_SendData(&emm_Data);
}

void Emm_Init(int Addr) {
	Emm_Data emm_Data;
	emm_Data.addr = Addr;
	emm_Data.funcCode = 0xF3;
	unsigned char temp[] = {0xAB, 0x01, 0x00};
	emm_Data.dataSize = 3;
	emm_Data.data = temp;
	emm_Data.checkSum = 0x6B;
	Emm_SendData(&emm_Data);
}
