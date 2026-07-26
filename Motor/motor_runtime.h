#ifndef MOTOR_RUNTIME_H
#define MOTOR_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

#include "Motor/newmotor_driver.h"
#include "Motor/newmotor_speed_ctrl.h"

void MotorRuntime_Init(void);
/**
 * @brief 执行一次编码器采样与速度闭环更新。
 *
 * @return true 表示本次实际完成了一次 10 ms 控制周期；false 表示尚未到更新时刻。
 */
bool MotorRuntime_Update(uint32_t nowTime, uint32_t nowUs);
void MotorRuntime_SetTargetWheelMmps(float left_mmps, float right_mmps);
void MotorRuntime_SetTargetRobot(float vx_mmps, float wz_radps);
/** @brief 同时设置左右轮速度环 PID 参数。 */
void MotorRuntime_SetPid(float kp, float ki, float kd);
/** @brief 读取当前双轮共用的速度环 PID 参数。 */
void MotorRuntime_GetPid(float *kp, float *ki, float *kd);
void MotorRuntime_Stop(NewMotor_StopMode stop_mode);
/** @brief 清空 PID、编码器和里程，并以指定模式停车。 */
void MotorRuntime_ResetAndStop(NewMotor_StopMode stop_mode);
void MotorRuntime_ResetDistance(void);
float MotorRuntime_GetLeftDistanceMm(void);
float MotorRuntime_GetRightDistanceMm(void);
float MotorRuntime_GetAverageDistanceMm(void);
/** @brief 读取最近一次控制周期计算出的左右轮实测速度（单位：mm/s）。 */
void MotorRuntime_GetMeasuredWheelMmps(float *left_mmps, float *right_mmps);
NewMotor_SpeedCtrl *MotorRuntime_GetController(void);

#endif
