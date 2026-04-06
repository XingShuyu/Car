
/**
 * @file    ultrasonic.c
 * @brief   超声波驱动实现，依赖 SysTick 提供 micros() 函数
 * @note    需要外部提供：
 *           - SYSCFG_DL_init() 已初始化引脚复用（GPIOB.17, GPIOB.19）
 *           - SysTick 每 1us 中断一次，并递增全局变量 systick_us
 *           - 或者实现 micros() 函数返回微秒计数
 */

#include "ultrasonic.h"
#include "ti_msp_dl_config.h"
#include "BasicMicroLib/getTime.h"          // 提供 getNowUs(), getTimeUs()
#include "BasicMicroLib/delay.h"
#include <stddef.h>

/* ------------------------------------------------------------------
   引脚定义（使用你生成的宏）
------------------------------------------------------------------ */
#define TRIG_PORT           GPIOB
#define TRIG_PIN            Distance_Trig_PIN       // DL_GPIO_PIN_19

#define ECHO_PORT           GPIOB
#define ECHO_PIN            Distance_Echo_PIN       // DL_GPIO_PIN_17

/* ------------------------------------------------------------------
   超声波参数
------------------------------------------------------------------ */
#define SOUND_SPEED_CM_PER_US   0.0343f     // 声速 cm/μs，20°C
#define MAX_DISTANCE_CM          20.0f     // 最大测量距离 0.2m
#define TIMEOUT_US              23300.0f   // 约 23300μs
void Ultrasonic_Init(void) {
    // 配置 Trig 为输出，初始低电平
    DL_GPIO_initDigitalOutput(Distance_Trig_IOMUX);
    DL_GPIO_clearPins(TRIG_PORT, TRIG_PIN);

    // 配置 Echo 为输入（无需上拉）
    DL_GPIO_initDigitalInput(Distance_Echo_IOMUX);
}
float Ultrasonic_GetDistance(void) {
    uint32_t start_us, end_us;

    /* 1. 发送 12μs 高电平触发脉冲 */
    DL_GPIO_setPins(TRIG_PORT, TRIG_PIN);
    uint32_t start0 = getNowUs();
    while (getTimeUs(getNowUs(), start0) < 12);
    DL_GPIO_clearPins(TRIG_PORT, TRIG_PIN);

    /* 2. 等待 Echo 变为高电平（模块开始发射） */
    uint32_t timeout_start = getNowUs();
    while (DL_GPIO_readPins(ECHO_PORT, ECHO_PIN) == 0) {
        if (getTimeUs(getNowUs(), timeout_start) > TIMEOUT_US) {
            return 0.0f;   // 超时无回波
        }
    }
    start_us = getNowUs();

    /* 3. 等待 Echo 变为低电平（回波接收完成） */
    while (DL_GPIO_readPins(ECHO_PORT, ECHO_PIN) != 0) {
        if (getTimeUs(getNowUs(), start_us) > TIMEOUT_US) {
            return 0.0f;
        }
    }
    end_us = getNowUs();

    /* 4. 计算距离（getTimeUs 安全处理回绕） */
    uint32_t duration_us = getTimeUs(end_us, start_us);
    float distance = (float)duration_us * SOUND_SPEED_CM_PER_US / 2.0f;
    printf("duration_us = %lu us, distance = %.2f cm\n", duration_us, distance);
    
    return distance;
}