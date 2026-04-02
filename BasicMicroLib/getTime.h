#ifndef GET_TIME_H
#define GET_TIME_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

extern volatile uint32_t startTime;
extern volatile uint32_t nowTime;

// 使能 SysTick 中断，用于扩展 24 位计数器
void TimeBase_Init(void);

// 读取当前时间戳（单位 us/ms）
uint32_t getNowUs(void);
uint32_t getNowMs(void);

// 输入当前时间戳与上次时间戳，返回经过时间（自动处理 uint32 回绕）
uint32_t getTimeUs(uint32_t nowUs, uint32_t lastUs);
uint32_t getTimeMs(uint32_t nowMs, uint32_t lastMs);

#endif