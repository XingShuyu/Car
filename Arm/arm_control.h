#ifndef ARM_ARM_CONTROL_H
#define ARM_ARM_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "Arm/jibot_servo.h"

/** 单次读取到的四轴原始 PWM 状态。 */
typedef struct ArmControl_PwmState {
	uint16_t pwm[JIBOT_SERVO_COUNT];
	/* 位 n 为 1 表示 pwm[n] 已由机械臂的有效应答更新。 */
	uint8_t validMask;
} ArmControl_PwmState;

/** 手动示教模式中单轴位置查询的超时时间。 */
#define ARM_CONTROL_READ_TIMEOUT_MS (50U)

/**
 * @brief 依次读取机械臂四个关节的原始 PWM 位置。
 *
 * 本函数为阻塞式操作，最久等待 JIBOT_SERVO_COUNT * timeout_ms。
 * 查询失败的关节对应 validMask 位为 0，pwm 中的数值不会被调用方使用。
 *
 * @param state 非空，函数返回前写入本次读取结果。
 * @param timeout_ms 每个关节等待位置应答的最长时间，单位 ms。
 * @return 四个关节均读取成功返回 true；任一关节超时或参数无效返回 false。
 */
bool ArmControl_ReadAllPwm(ArmControl_PwmState *state, uint16_t timeout_ms);

/**
 * @brief 进入手动示教测试：B2 读取并锁定/再次释放，B1 退出。
 *
 * 进入时立即释放四轴扭力，用户可手动摆放。B2 按下后先记录各轴 PWM、
 * 再恢复扭力并显示结果；之后 B2 会再次释放扭力。无论在何种状态按 B1
 * 退出，函数都会尝试恢复全部关节扭力。
 *
 * @note 运行前必须支撑好机械臂，释放扭力后关节可能在重力作用下落下。
 */
void ArmControl_RunTeachTest(void);

#endif /* ARM_ARM_CONTROL_H */
