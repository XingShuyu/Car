#include "ti_msp_dl_config.h"
#include "BasicMicroLib/delay.h"
#include "BasicMicroLib/usart.h"
#include "GrayScale/Grayscale_Scan.c"
#include "Motor/motor.h"
#include "MCU6050/mpu6050.h"
#include "BasicMicroLib/getTime.h"
#include <stdio.h>
#include <stdbool.h>

//循迹pid
volatile PID garyscalePid = {1.0f,1.0f,1.0f,100.0,0};

//-------------------
//各种时间声明
//获取电机速度时间戳
volatile uint32_t lastMotorSpeedTime=0;
//数据输出时间戳
volatile uint32_t lastUartTime=0;
