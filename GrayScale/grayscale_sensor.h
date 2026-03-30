#ifndef __GRAYSCALE_SENSOR_H
#define __GRAYSCALE_SENSOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "BasicMicroLib/delay.h"
#include "BasicMicroLib/usart.h"

//=====================================================================================
//  引脚配置接口 (Pin Configuration)
//=====================================================================================
// --- 通道选择引脚定义 (AD0, AD1, AD2)  Channel selection pin definition (AD0, AD1, AD2)---
#define SENSOR_AD0_PORT         GrayS_AD0_PORT
#define SENSOR_AD0_PIN          GrayS_AD0_PIN

#define SENSOR_AD1_PORT         GrayS_AD1_PORT
#define SENSOR_AD1_PIN          GrayS_AD1_PIN

#define SENSOR_AD2_PORT         GrayS_AD2_PORT
#define SENSOR_AD2_PIN          GrayS_AD2_PIN

//=====================================================================================
//  GPIO操作抽象接口 (GPIO Operation Macros)
//=====================================================================================
#define GRAYSCALE_PIN_WRITE(port, pin, state) do { \
    if(state) DL_GPIO_setPins(port, pin); \
    else DL_GPIO_clearPins(port, pin); \
} while(0)

#define SENSOR_AD0_WRITE(state)  GRAYSCALE_PIN_WRITE(SENSOR_AD0_PORT, SENSOR_AD0_PIN, state)
#define SENSOR_AD1_WRITE(state)  GRAYSCALE_PIN_WRITE(SENSOR_AD1_PORT, SENSOR_AD1_PIN, state)
#define SENSOR_AD2_WRITE(state)  GRAYSCALE_PIN_WRITE(SENSOR_AD2_PORT, SENSOR_AD2_PIN, state)

#define SENSOR_OUT_READ()        (!!(DL_GPIO_readPins(GrayS_OUT_PORT, GrayS_OUT_PIN)))

//=====================================================================================
//  驱动函数接口 (Driver API)
//=====================================================================================

#define GRAYSCALE_SENSOR_CHANNELS   8   // 传感器通道总数 Number of sensor channels

void Grayscale_Sensor_Init(void);
void Grayscale_Sensor_Read_All(uint16_t* sensor_values);
void Grayscale_Sensor_Read_Main(uint16_t* sensor_values);
void Grayscale_Sensor_Read_Other(uint16_t* sensor_values);
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel);

#endif // __GRAYSCALE_SENSOR_H
