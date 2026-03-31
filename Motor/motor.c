#include "ti_msp_dl_config.h"
#include "motor.h"

void Motor_Init(void) {
    // PWM 和 GPIO 已在 SysConfig 中初始化，这里确保电机初始处于停止状态
    Motor_Brake();
}

void Motor_SetSpeed(int16_t left_speed, int16_t right_speed) {
    uint32_t left_pwm = 0;
    uint32_t right_pwm = 0;

    /*----------------- 处理右轮速度 -----------------*/
    if (right_speed > 0) {
        // 正转：方向引脚置高
        DL_GPIO_setPins(MotorContor_PORT, MotorContor_MotorRight_PIN);
        right_pwm = (uint32_t)right_speed;
    } else if (right_speed < 0) {
        // 反转：方向引脚置低
        DL_GPIO_clearPins(MotorContor_PORT, MotorContor_MotorRight_PIN);
        right_pwm = (uint32_t)(-right_speed);
    } else {
        right_pwm = 0;
    }
    
    // 限幅保护
    if(right_pwm > MOTOR_MAX_SPEED) right_pwm = MOTOR_MAX_SPEED;
    // 设置右轮PWM占空比 (通道0对应PB21)
    DL_TimerG_setCaptureCompareValue(TIM_1_INST, right_pwm, GPIO_TIM_1_C0_IDX);

    /*----------------- 处理左轮速度 -----------------*/
    if (left_speed > 0) {
        // 正转：方向引脚置高
        DL_GPIO_setPins(MotorContor_PORT, MotorContor_MotorLeft_PIN);
        left_pwm = (uint32_t)left_speed;
    } else if (left_speed < 0) {
        // 反转：方向引脚置低
        DL_GPIO_clearPins(MotorContor_PORT, MotorContor_MotorLeft_PIN);
        left_pwm = (uint32_t)(-left_speed);
    } else {
        left_pwm = 0;
    }

    // 限幅保护
    if(left_pwm > MOTOR_MAX_SPEED) left_pwm = MOTOR_MAX_SPEED;
    // 设置左轮PWM占空比 (通道1对应PA30)
    DL_TimerG_setCaptureCompareValue(TIM_1_INST, left_pwm, GPIO_TIM_1_C1_IDX);
}

void Motor_Brake(void) {
    // 将PWM占空比设为0
    DL_TimerG_setCaptureCompareValue(TIM_1_INST, 0, GPIO_TIM_1_C0_IDX);
    DL_TimerG_setCaptureCompareValue(TIM_1_INST, 0, GPIO_TIM_1_C1_IDX);
    
    // D157B完整刹车逻辑通常是将IN1和IN2设为相同电平。
    // 因为您只有一个方向引脚，这里仅做停止处理。
    DL_GPIO_clearPins(MotorContor_PORT, MotorContor_MotorRight_PIN);
    DL_GPIO_clearPins(MotorContor_PORT, MotorContor_MotorLeft_PIN);
}