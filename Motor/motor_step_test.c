#include "Motor/motor_step_test.h"

#if MOTOR_STEP_TEST_ENABLE

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "BasicMicroLib/delay.h"
#include "BasicMicroLib/getTime.h"
#include "BasicMicroLib/usart.h"
#include "Motor/motor_encoder.h"
#include "Motor/motor_runtime.h"
#include "OLED/display.h"

static bool MotorStepTest_IsReached(float measured_mmps, float target_mmps)
{
	float absTarget = fabsf(target_mmps);
	float absMeasured = fabsf(measured_mmps);
	float tolerance = absTarget * (1.0f - MOTOR_STEP_TEST_REACHED_RATIO);

	if (tolerance < MOTOR_STEP_TEST_ABS_TOL_MMPS) {
		tolerance = MOTOR_STEP_TEST_ABS_TOL_MMPS;
	}

	return fabsf(absTarget - absMeasured) <= tolerance;
}

void MotorStepTest_Run(void)
{
	NewMotor_SpeedCtrl *motor = MotorRuntime_GetController();
	const float targetMmps = MOTOR_STEP_TEST_TARGET_MMPS;
	uint32_t commandTimeUs;
	uint32_t firstMoveTimeUs = 0;
	uint32_t reachedTimeUs = 0;
	uint32_t stableStartTimeUs = 0;
	uint32_t lastUpdateUs;
	uint32_t lastLogMs;
	uint32_t updateCount = 0;
	uint32_t lastActualPeriodUs = 0;
	uint32_t measureWindowUs = 0;
	int32_t measureLeftCount = 0;
	int32_t measureRightCount = 0;
	int32_t lastLeftCount = 0;
	int32_t lastRightCount = 0;
	int32_t totalLeftCount = 0;
	int32_t totalRightCount = 0;
	float lastLeftMmps = 0.0f;
	float lastRightMmps = 0.0f;
	float lastAvgMmps = 0.0f;
	float lastWindowLeftMmps = 0.0f;
	float lastWindowRightMmps = 0.0f;
	float lastWindowAvgMmps = 0.0f;
	int16_t lastLeftPwm = 0;
	int16_t lastRightPwm = 0;
	bool firstMoveSeen = false;
	bool stableTiming = false;
	bool reached = false;
	bool pwmSaturated = false;

	MotorEncoder_ResetCounts();
	MotorRuntime_ResetDistance();

	MotorRuntime_SetTargetWheelMmps(0.0f, 0.0f);
	MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
	delay_ms(300);

	Display_Clear();
	Display_ShowString(0, 0, "Motor Step Test");
	{
		char oledLine[22];
		snprintf(oledLine, sizeof(oledLine), "Target:%d mm/s", (int)targetMmps);
		Display_ShowString(2, 0, oledLine);
	}
	Display_ShowString(4, 0, "Measuring...");

	MotorEncoder_ResetCounts();

	commandTimeUs = getNowUs();
	lastUpdateUs = commandTimeUs;
	lastLogMs = getNowMs();
	MotorRuntime_SetTargetWheelMmps(targetMmps, targetMmps);

	while (getTimeUs(getNowUs(), commandTimeUs) <
		   (MOTOR_STEP_TEST_TIMEOUT_MS * 1000U)) {
		uint32_t nowUs = getNowUs();
		uint32_t nowMs = getNowMs();

		USART_PollTx();

		if (getTimeUs(nowUs, lastUpdateUs) >=
			MOTOR_STEP_TEST_CONTROL_PERIOD_US) {
			int32_t leftCountSnapshot;
			int32_t rightCountSnapshot;
			float leftMmps;
			float rightMmps;
			float avgMmps;
			int16_t leftPwm;
			int16_t rightPwm;
			bool measureWindowReady = false;

			motor->sample_period_s =
				(float)getTimeUs(nowUs, lastUpdateUs) / 1000000.0f;
			lastActualPeriodUs = getTimeUs(nowUs, lastUpdateUs);
			lastUpdateUs = nowUs;
			updateCount++;

			MotorEncoder_ReadAndClear(&leftCountSnapshot, &rightCountSnapshot);

			totalLeftCount += leftCountSnapshot;
			totalRightCount += rightCountSnapshot;
			measureLeftCount += leftCountSnapshot;
			measureRightCount += rightCountSnapshot;
			measureWindowUs += lastActualPeriodUs;

			if (!firstMoveSeen &&
				(leftCountSnapshot != 0 || rightCountSnapshot != 0)) {
				firstMoveSeen = true;
				firstMoveTimeUs = nowUs;
			}

			NewMotorSpeedCtrl_UpdateByEncoderDelta(motor, leftCountSnapshot,
												   rightCountSnapshot);
			NewMotorSpeedCtrl_GetMeasuredWheelMmps(motor, &leftMmps,
												   &rightMmps);
			NewMotorSpeedCtrl_GetOutputPwmTicks(motor, &leftPwm, &rightPwm);
			avgMmps = NewMotor_LeftRightToLinearSpeedMmps(leftMmps, rightMmps);
			lastLeftCount = leftCountSnapshot;
			lastRightCount = rightCountSnapshot;
			lastLeftMmps = leftMmps;
			lastRightMmps = rightMmps;
			lastAvgMmps = avgMmps;
			lastLeftPwm = leftPwm;
			lastRightPwm = rightPwm;
			if (leftPwm >= MOTOR_STEP_TEST_PWM_SAT_TICKS ||
				leftPwm <= -MOTOR_STEP_TEST_PWM_SAT_TICKS ||
				rightPwm >= MOTOR_STEP_TEST_PWM_SAT_TICKS ||
				rightPwm <= -MOTOR_STEP_TEST_PWM_SAT_TICKS) {
				pwmSaturated = true;
			}

			if (measureWindowUs >= MOTOR_STEP_TEST_MEASURE_WINDOW_US) {
				float measureWindowS = (float)measureWindowUs / 1000000.0f;
				lastWindowLeftMmps =
					NewMotor_EncoderDeltaToDistanceMm(measureLeftCount) /
					measureWindowS;
				lastWindowRightMmps =
					NewMotor_EncoderDeltaToDistanceMm(measureRightCount) /
					measureWindowS;
				lastWindowAvgMmps = NewMotor_LeftRightToLinearSpeedMmps(
					lastWindowLeftMmps, lastWindowRightMmps);
				measureLeftCount = 0;
				measureRightCount = 0;
				measureWindowUs = 0;
				measureWindowReady = true;
			}

			if (measureWindowReady) {
				if (MotorStepTest_IsReached(lastWindowAvgMmps, targetMmps)) {
					if (!stableTiming) {
						stableTiming = true;
						stableStartTimeUs = nowUs;
						reachedTimeUs = nowUs;
					}
					if (getTimeUs(nowUs, stableStartTimeUs) >=
						(MOTOR_STEP_TEST_STABLE_TIME_MS * 1000U)) {
						reached = true;
						break;
					}
				} else {
					stableTiming = false;
				}
			}

			if (getTimeMs(nowMs, lastLogMs) >=
				MOTOR_STEP_TEST_LOG_INTERVAL_MS) {
				lastLogMs = nowMs;
				printf("Step t=%lu ms L=%d R=%d Avg=%d WinAvg=%d PWM=%d,%d "
					   "Cnt=%ld,%ld dt=%lu us\n",
					   (unsigned long)(getTimeUs(nowUs, commandTimeUs) / 1000U),
					   (int)leftMmps, (int)rightMmps, (int)avgMmps,
					   (int)lastWindowAvgMmps, leftPwm, rightPwm,
					   (long)leftCountSnapshot, (long)rightCountSnapshot,
					   (unsigned long)lastActualPeriodUs);
			}
		}
	}

	MotorRuntime_SetTargetWheelMmps(0.0f, 0.0f);
	MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);

	if (reached) {
		uint32_t commandToTargetMs =
			getTimeUs(reachedTimeUs, commandTimeUs) / 1000U;
		uint32_t motorStartToTargetMs =
			firstMoveSeen ? (getTimeUs(reachedTimeUs, firstMoveTimeUs) / 1000U)
						  : 0U;
		char oledLine[22];

		Display_Clear();
		Display_ShowString(0, 0, "Step Test Done");
		Display_ShowString(2, 0, "Reached");
		snprintf(oledLine, sizeof(oledLine), "Cmd:%lu ms",
				 (unsigned long)commandToTargetMs);
		Display_ShowString(4, 0, oledLine);
		if (firstMoveSeen) {
			snprintf(oledLine, sizeof(oledLine), "Run:%lu ms",
					 (unsigned long)motorStartToTargetMs);
			Display_ShowString(6, 0, oledLine);
		} else {
			Display_ShowString(6, 0, "Run:no edge");
		}
		snprintf(oledLine, sizeof(oledLine), "PWM:%s",
				 pwmSaturated ? "Hit 2000" : "No limit");
		Display_ShowString(7, 0, oledLine);
	} else {
		char oledLine[22];
		uint32_t elapsedMs = getTimeUs(getNowUs(), commandTimeUs) / 1000U;
		float oneCountMmps =
			NewMotor_EncoderDeltaToDistanceMm(1) /
			((float)MOTOR_STEP_TEST_CONTROL_PERIOD_US / 1000000.0f);
		float oneCountWindowMmps =
			NewMotor_EncoderDeltaToDistanceMm(1) /
			((float)MOTOR_STEP_TEST_MEASURE_WINDOW_US / 1000000.0f);

		Display_Clear();
		Display_ShowString(0, 0, "Step Test Done");
		Display_ShowString(2, 0, "Timeout");
		snprintf(oledLine, sizeof(oledLine), "T:%d A:%d", (int)targetMmps,
				 (int)lastWindowAvgMmps);
		Display_ShowString(4, 0, oledLine);
		snprintf(oledLine, sizeof(oledLine), "PWM:%d,%d", lastLeftPwm,
				 lastRightPwm);
		Display_ShowString(6, 0, oledLine);
		snprintf(oledLine, sizeof(oledLine), "PWM:%s",
				 pwmSaturated ? "Hit 2000" : "No limit");
		Display_ShowString(7, 0, oledLine);

		printf("Step Timeout target=%d elapsed=%lu ms updates=%lu last_dt=%lu "
			   "us\n",
			   (int)targetMmps, (unsigned long)elapsedMs,
			   (unsigned long)updateCount, (unsigned long)lastActualPeriodUs);
		printf("Step Final instant_left=%d instant_right=%d instant_avg=%d "
			   "window_left=%d window_right=%d window_avg=%d pwm=%d,%d "
			   "last_cnt=%ld,%ld total_cnt=%ld,%ld\n",
			   (int)lastLeftMmps, (int)lastRightMmps, (int)lastAvgMmps,
			   (int)lastWindowLeftMmps, (int)lastWindowRightMmps,
			   (int)lastWindowAvgMmps, lastLeftPwm, lastRightPwm,
			   (long)lastLeftCount, (long)lastRightCount, (long)totalLeftCount,
			   (long)totalRightCount);
		printf("Step PWM saturated=%s threshold=%d\n",
			   pwmSaturated ? "yes" : "no", MOTOR_STEP_TEST_PWM_SAT_TICKS);
		printf(
			"Step Resolution control_period=%lu us one_count=%d mm/s "
			"measure_window=%lu us window_one_count=%d mm/s dropped_tx=%lu\n",
			(unsigned long)MOTOR_STEP_TEST_CONTROL_PERIOD_US,
			(int)(oneCountMmps + 0.5f),
			(unsigned long)MOTOR_STEP_TEST_MEASURE_WINDOW_US,
			(int)(oneCountWindowMmps + 0.5f),
			(unsigned long)USART_GetDroppedTxBytes());
	}

	while (1) {
		USART_PollTx();
	}
}
#endif
