#include "ti_msp_dl_config.h"
#include "BasicMicroLib/delay.h"
#include "BasicMicroLib/usart.h"
#include "GrayScale/Grayscale_Scan.c"  // 修改1：.c 改为 .h
#include "Motor/motor.h"
#include <stdio.h>

volatile uint16_t grayscale[8];

int main(void)
{
    /* 
     * 关键修复：在系统初始化前加入延时。
     * MSPM0在自动生成的 SYSCFG_DL_init() 中会复位GPIOA，这会导致SWD调试引脚瞬时重置。
     * 如果上电后代码立即执行到这里，SWD连接会被切断，导致下载器无法连接并报错“锁死”。
     * 添加1-2秒的延时，能给调试器留出充足的连接和暂停CPU的时间。
     */
    for(volatile uint32_t i = 0; i < 3200000; i++); 

    SYSCFG_DL_init(); // 由SysConfig自动生成的初始化函数
    USART_Init();     // 使能UART中断（接收依赖此步骤）    
    /* 
     * 修改2（最关键）：必须同时启动两个定时器！
     * 根据你的 SysConfig，左电机绑定了 TIMG8，右电机绑定了 TIMG6。
     * 如果宏名字报错，请去 ti_msp_dl_config.h 里搜索 TIMG 找到准确的名字
     * (也有可能被重命名为 PWM_MotorLeft_INST 等，取决于你SysConfig的命名)
     */
    setvbuf(stdout, NULL, _IONBF, 0);
    DL_TimerG_startCounter(MotorLeft_INST); 
    DL_TimerG_startCounter(MotorRight_INST); 

    // 打印启动信息
    printf("MSPM0G3507 D157B Motor Test Start!\r\n");
    PID garyscalePid = {0};
    garyscalePid.p = 1.0f;
    garyscalePid.i = 1.0f;
    garyscalePid.d = 1.0f;
    garyscalePid.i_Max = 100.0f;

    while (1)
    {
        // float out = Grayscale_Line((uint16_t *)grayscale, &garyscalePid);
        // printf("out = %.2f\r\n", out);
        bool out = Grayscale_Cross((uint16_t *)grayscale, (uint16_t)0.5, 1);
        printf("%d,now: %.2f",out,(double)grayscale[0]);
        delay_ms(500);
        
        /* 动作1：全速前进 */
        // Motor_SetSpeed(1000, 1000); 
        // delay_ms(2000);
        // Motor_Brake();
        // delay_ms(2000);
        // Motor_SetSpeed(-1000,-1000);
        // delay_ms(2000);
    }
}