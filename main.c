/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * 移植说明：
 * 本代码基于 TI MSPM0G3507 (Cortex-M0+)
 * 使用 DriverLib 开发
 * 请确保在 SysConfig 中配置了相应的引脚和时钟
 */

#include "ti_msp_dl_config.h"
#include "BasicMicroLib/delay.h"
#include "BasicMicroLib/usart.h"
#include "Motor/motor.h"
#include <stdio.h>

// // UART 重定向 fputc
// int fputc(int ch, FILE *fp) {
//     DL_UART_transmitData(UART_0_INST, (uint8_t)ch);
//     while (DL_UART_isBusy(UART_0_INST));
//     return ch;
// }

// // 简单的延时函数 (基于TIMA0，粗略延时)
// void delay_ms(uint32_t ms) {
//     for (uint32_t i = 0; i < ms; i++) {
//         // 如果您的系统时钟是32MHz或64MHz，这里需要根据实际情况循环
//         // 强烈建议使用SysTick或定时器中断来做精准延时
//         for(volatile uint32_t j=0; j<4000; j++); 
//     }
// }

int main(void)
{
    /* 
     * 关键修复：在系统初始化前加入延时。
     * MSPM0在自动生成的 SYSCFG_DL_init() 中会复位GPIOA，这会导致SWD调试引脚瞬时重置。
     * 如果上电后代码立即执行到这里，SWD连接会被切断，导致下载器无法连接并报错“锁死”。
     * 添加1-2秒的延时，能给调试器留出充足的连接和暂停CPU的时间。
     */asdasd
    for(volatile uint32_t i = 0; i < 3200000; i++); 

    SYSCFG_DL_init(); // 由SysConfig自动生成的初始化函数
    
    // 启动PWM定时器 (TIMG8)
    DL_TimerG_startCounter(TIM_1_INST);

    // 打印启动信息
    printf("MSPM0G3507 D157B Motor Test Start!\r\n");

    while (1)
    {
        /* 动作1：全速前进 */
        printf("Forward...\r\n");
        Motor_SetSpeed(2000, 2000); 
        delay_ms(2000);

        /* 动作2：停止 */
        printf("Stop...\r\n");
        Motor_Brake();
        delay_ms(1000);

        /* 动作3：原地左转 (右轮正转，左轮反转) */
        printf("Turn Left...\r\n");
        Motor_SetSpeed(-1500, 1500);
        delay_ms(2000);

        /* 动作4：原地右转 (左轮正转，右轮反转) */
        printf("Turn Right...\r\n");
        Motor_SetSpeed(1500, -1500);
        delay_ms(2000);

        /* 动作5：后退 */
        printf("Backward...\r\n");
        Motor_SetSpeed(-2000, -2000);
        delay_ms(2000);

        /* 动作6：缓慢停止 */
        printf("Slow Stop...\r\n");
        Motor_SetSpeed(0, 0);
        delay_ms(2000);
    }
}