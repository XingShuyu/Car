#include "ti_msp_dl_config.h"
#include "BasicMicroLib/delay.h"
#include "BasicMicroLib/usart.h"
#include "GrayScale/Grayscale_Scan.c"  // 修改1：.c 改为 .h
#include "Motor/motor.h"
#include <stdio.h>
#include <stdbool.h>
volatile uint16_t grayscale[8];
volatile int32_t motor1_count = 0;
volatile int32_t motor2_count = 0;
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
    // 开启 GPIOA 和 GPIOB 的全局中断 (因为编码器引脚跨越了这两个端口)
    NVIC_EnableIRQ(MotorMonitor_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(MotorMonitor_GPIOB_INT_IRQN);
    // 启动你的 TIM_0 定时器 (用于后续做 10ms/20ms 速度计算)
    // DL_Timer_startCounter(TIM_0_INST);
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
void GPIOA_IRQHandler(void) {
    // 获取当前触发中断的具体引脚 (掩码)
    uint32_t pending_pins = DL_GPIO_getPendingInterrupt(MotorMonitor_E1A_PORT);

    // --- 处理电机 1 (E1A 和 E1B 都在 PORTA) ---
    if (pending_pins & (MotorMonitor_E1A_PIN | MotorMonitor_E1B_PIN)) {
        bool m1_A = (DL_GPIO_readPins(MotorMonitor_E1A_PORT, MotorMonitor_E1A_PIN) != 0);
        bool m1_B = (DL_GPIO_readPins(MotorMonitor_E1B_PORT, MotorMonitor_E1B_PIN) != 0);

        if (pending_pins & MotorMonitor_E1A_PIN) {
            DL_GPIO_clearInterruptStatus(MotorMonitor_E1A_PORT, MotorMonitor_E1A_PIN);
            if (m1_A == m1_B) motor1_count++; 
            else              motor1_count--;
        }
        
        if (pending_pins & MotorMonitor_E1B_PIN) {
            DL_GPIO_clearInterruptStatus(MotorMonitor_E1B_PORT, MotorMonitor_E1B_PIN);
            if (m1_A != m1_B) motor1_count++; 
            else              motor1_count--;
        }
    }

    // --- 处理电机 2 的 E2B (在 PORTA) ---
    if (pending_pins & MotorMonitor_E2B_PIN) {
        DL_GPIO_clearInterruptStatus(MotorMonitor_E2B_PORT, MotorMonitor_E2B_PIN);
        
        // 注意：E2A 在 PORTB，需要跨端口读取它的状态
        bool m2_A = (DL_GPIO_readPins(MotorMonitor_E2A_PORT, MotorMonitor_E2A_PIN) != 0);
        bool m2_B = (DL_GPIO_readPins(MotorMonitor_E2B_PORT, MotorMonitor_E2B_PIN) != 0);

        if (m2_A != m2_B) motor2_count++; 
        else              motor2_count--;
    }
}

// ==========================================
// GPIOB 中断服务函数 (处理 E2A)
// ==========================================
void GPIOB_IRQHandler(void) {
    uint32_t pending_pins = DL_GPIO_getPendingInterrupt(MotorMonitor_E2A_PORT);

    // --- 处理电机 2 的 E2A (在 PORTB) ---
    if (pending_pins & MotorMonitor_E2A_PIN) {
        DL_GPIO_clearInterruptStatus(MotorMonitor_E2A_PORT, MotorMonitor_E2A_PIN);
        
        // 注意：E2B 在 PORTA，需要跨端口读取它的状态
        bool m2_A = (DL_GPIO_readPins(MotorMonitor_E2A_PORT, MotorMonitor_E2A_PIN) != 0);
        bool m2_B = (DL_GPIO_readPins(MotorMonitor_E2B_PORT, MotorMonitor_E2B_PIN) != 0);

        if (m2_A == m2_B) motor2_count++; 
        else              motor2_count--;
    }
}