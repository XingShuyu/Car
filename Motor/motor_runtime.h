#ifndef MOTOR_RUNTIME_H
#define MOTOR_RUNTIME_H

#include <stdint.h>

#include "Motor/newmotor_driver.h"
#include "Motor/newmotor_speed_ctrl.h"

void MotorRuntime_Init(void);
void MotorRuntime_Update(uint32_t nowTime, uint32_t nowUs);
void MotorRuntime_SetTargetWheelMmps(float left_mmps, float right_mmps);
void MotorRuntime_SetTargetRobot(float vx_mmps, float wz_radps);
void MotorRuntime_Stop(NewMotor_StopMode stop_mode);
void MotorRuntime_ResetDistance(void);
float MotorRuntime_GetLeftDistanceMm(void);
float MotorRuntime_GetRightDistanceMm(void);
float MotorRuntime_GetAverageDistanceMm(void);
NewMotor_SpeedCtrl *MotorRuntime_GetController(void);

#endif
