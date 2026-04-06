/**
 * @file    ultrasonic.h
 * @brief   超声波模块 HC-SR04 驱动（GPIOB.17 Echo, GPIOB.19 Trig）
 * @details 使用轮询 + SysTick 微秒延时测量距离
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include <stdint.h>
#include <stdbool.h>

/* 初始化超声波模块（配置 GPIO） */
void Ultrasonic_Init(void);

/* 获取距离（单位：cm），失败或超时返回 0.0f */
float Ultrasonic_GetDistance(void);

#endif /* ULTRASONIC_H_ */