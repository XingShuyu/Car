#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

// PWM最大计数值 (对应SysConfig中的 3200)
#define PWM_MAX_COUNT      3200
#define MOTOR_MAX_SPEED    3000  // 留一点余量防止占空比100%导致驱动异常

/**
 * @brief 初始化电机驱动相关引脚
 */
void Motor_Init(void);

/**
 * @brief 设置左右电机速度
 * @param left_speed  左轮速度 (-MOTOR_MAX_SPEED 到 +MOTOR_MAX_SPEED)
 * @param right_speed 右轮速度 (-MOTOR_MAX_SPEED 到 +MOTOR_MAX_SPEED)
 * @note 正数代表正转，负数代表反转，0代表停止
 */
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed);

/**
 * @brief 刹车停止 (将PWM拉低，方向引脚保持)
 */
void Motor_Brake(void);

#endif // MOTOR_H