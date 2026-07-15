#include "Motor/motor_runtime.h"

#include <stdio.h>

#include "BasicMicroLib/getTime.h"
#include "Motor/motor_encoder.h"
#include "ti_msp_dl_config.h"

#define MOTOR_RUNTIME_UPDATE_INTERVAL_US 10000U
#define MOTOR_RUNTIME_LOG_INTERVAL_MS 100U

// 获取电机速度时间戳
static uint32_t lastMotorSpeedTime = 0;
// 数据输出时间戳
static uint32_t lastUartTime = 0;

static NewMotor_SpeedCtrl motor;
static int32_t leftDistance = 0;
static int32_t rightDistance = 0;

void MotorRuntime_Init(void)
{
	// 电机初始化
	DL_TimerG_startCounter(MotorLeft_INST);
	DL_TimerG_startCounter(MotorRight_INST);
	NewMotorSpeedCtrl_Init(&motor, 0.001f);
	NewMotorSpeedCtrl_SetPid(&motor, 3.0, 0.6, 0.002);
	NewMotorSpeedCtrl_SetOutputLimit(&motor, -2000, 2000);
	// NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, BaseSpeed, BaseSpeed);

	MotorEncoder_ResetCounts();
	MotorRuntime_ResetDistance();
	lastMotorSpeedTime = 0;
	lastUartTime = 0;
}

void MotorRuntime_Update(uint32_t nowTime, uint32_t nowUs)
{
	if (getTimeUs(nowUs, lastMotorSpeedTime) > MOTOR_RUNTIME_UPDATE_INTERVAL_US) {
		int32_t leftCountSnapshot;
		int32_t rightCountSnapshot;
		int sped;

		motor.sample_period_s =
			(float)getTimeUs(nowUs, lastMotorSpeedTime) / 1000000.0f;
		lastMotorSpeedTime = nowUs;

		MotorEncoder_ReadAndClear(&leftCountSnapshot, &rightCountSnapshot);
		leftDistance += leftCountSnapshot;
		rightDistance += rightCountSnapshot;

		sped =
			(int)(NewMotor_EncoderDeltaToDistanceMm(leftCountSnapshot) /
				  motor.sample_period_s);
		sped = (int)(NewMotor_EncoderDeltaToDistanceMm(rightCountSnapshot) /
					 motor.sample_period_s);
		if (getTimeMs(nowTime, lastUartTime) >=
			MOTOR_RUNTIME_LOG_INTERVAL_MS) {
			lastUartTime = nowTime;
		}

		NewMotorSpeedCtrl_UpdateByEncoderDelta(&motor, leftCountSnapshot,
											   rightCountSnapshot);
	}
}

void MotorRuntime_SetTargetWheelMmps(float left_mmps, float right_mmps)
{
	NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, left_mmps, right_mmps);
}

void MotorRuntime_SetTargetRobot(float vx_mmps, float wz_radps)
{
	NewMotorSpeedCtrl_SetTargetRobot(&motor, vx_mmps, wz_radps);
}

void MotorRuntime_Stop(NewMotor_StopMode stop_mode)
{
	NewMotor_Stop(stop_mode);
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

NewMotor_SpeedCtrl *MotorRuntime_GetController(void)
{
	return &motor;
}
