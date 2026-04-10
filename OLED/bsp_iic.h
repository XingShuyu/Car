#ifndef __BSP_IIC_H_
#define __BSP_IIC_H_

#include "ti_msp_dl_config.h"

#define OLED_IIC_ADDR  0x3C
void OLED_WR_Byte(uint8_t addr, uint8_t dat);
uint8_t OLED_WR_ReadByte(uint8_t addr);

#endif

