#include "Motor/motor_runtime.h"

#include "BasicMicroLib/getTime.h"
#include "Motor/motor_encoder.h"
#include "ti_msp_dl_config.h"

#define MOTOR_RUNTIME_UPDATE_INTERVAL_US 10000U

/* 获取电机速度时间戳。 */
static uint32_t lastMotorSpeedTime = 0;

static NewMotor_SpeedCtrl motor;
static int32_t leftDistance = 0;
static int32_t rightDistance = 0;

void MotorRuntime_Init(void)
{
	// 电机初始化
	DL_TimerG_startCounter(MotorLeft_INST);
	DL_TimerG_startCounter(MotorRight_INST);
	NewMotorSpeedCtrl_Init(&motor, 0.001f);
	NewMotorSpeedCtrl_SetPid(&motor, 3.0, 20.0, 0.002);
	NewMotorSpeedCtrl_SetOutputLimit(&motor, -2000, 2000);
	// NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, BaseSpeed, BaseSpeed);

	MotorEncoder_ResetCounts();
	MotorRuntime_ResetDistance();
	lastMotorSpeedTime = 0;
}

bool MotorRuntime_Update(uint32_t nowTime, uint32_t nowUs)
{
	(void)nowTime;

	if (getTimeUs(nowUs, lastMotorSpeedTime) >=
		MOTOR_RUNTIME_UPDATE_INTERVAL_US) {
		int32_t leftCountSnapshot;
		int32_t rightCountSnapshot;

		motor.sample_period_s =
			(float)getTimeUs(nowUs, lastMotorSpeedTime) / 1000000.0f;
		lastMotorSpeedTime = nowUs;

		MotorEncoder_ReadAndClear(&leftCountSnapshot, &rightCountSnapshot);
		leftDistance += leftCountSnapshot;
		rightDistance += rightCountSnapshot;

		NewMotorSpeedCtrl_UpdateByEncoderDelta(&motor, leftCountSnapshot,
											   rightCountSnapshot);
		return true;
	}

	return false;
}

void MotorRuntime_SetTargetWheelMmps(float left_mmps, float right_mmps)
{
	NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, left_mmps, right_mmps);
}

void MotorRuntime_SetTargetRobot(float vx_mmps, float wz_radps)
{
	NewMotorSpeedCtrl_SetTargetRobot(&motor, vx_mmps, wz_radps);
}

void MotorRuntime_SetPid(float kp, float ki, float kd)
{
	NewMotorSpeedCtrl_SetPid(&motor, kp, ki, kd);
}

void MotorRuntime_GetPid(float *kp, float *ki, float *kd)
{
	if (kp != NULL) {
		*kp = motor.pid_left.kp;
	}
	if (ki != NULL) {
		*ki = motor.pid_left.ki;
	}
	if (kd != NULL) {
		*kd = motor.pid_left.kd;
	}
}

void MotorRuntime_Stop(NewMotor_StopMode stop_mode)
{
	NewMotor_Stop(stop_mode);
}

void MotorRuntime_ResetAndStop(NewMotor_StopMode stop_mode)
{
	NewMotorSpeedCtrl_ResetAndStop(&motor, stop_mode);
	MotorEncoder_ResetCounts();
	MotorRuntime_ResetDistance();
	lastMotorSpeedTime = getNowUs();
}

void MotorRuntime_ResetDistance(void)
{
	leftDistance = 0;
	rightDistance = 0;
}

float MotorRuntime_GetLeftDistanceMm(void)
{
	return NewMotor_EncoderDeltaToDistanceMm(leftDistance);
}

float MotorRuntime_GetRightDistanceMm(void)
{
	return NewMotor_EncoderDeltaToDistanceMm(rightDistance);
}

float MotorRuntime_GetAverageDistanceMm(void)
{
	return 0.5f *
		   (MotorRuntime_GetLeftDistanceMm() + MotorRuntime_GetRightDistanceMm());
}

void MotorRuntime_GetMeasuredWheelMmps(float *left_mmps, float *right_mmps)
{
	NewMotorSpeedCtrl_GetMeasuredWheelMmps(&motor, left_mmps, right_mmps);
}

NewMotor_SpeedCtrl *MotorRuntime_GetController(void)
{
	return &motor;
}
