#include "Stage/stage_runner.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "Arm/arm_ik_motion.h"
#include "Arm/arm_motion_state.h"
#include "BasicMicroLib/PID.h"
#include "BasicMicroLib/getTime.h"
#include "Communication/maixcam_protocol.h"
#include "Communication/maixcam_serial.h"
#include "Drivers/button_select.h"
#include "Drivers/buzzer.h"
#include "GrayScale/Grayscale_Scan.h"
#include "GrayScale/grayscale_sensor.h"
#include "IMU/imu.h"
#include "Motor/motor_runtime.h"
#include "OLED/display.h"

#define STAGE_RUNNER_UPDATE_INTERVAL_MS 10U
#define STAGE_RUNNER_MAIXCAM_FRAME_SIZE 32U
#define STAGE_RUNNER_MAIXCAM_NOTIFY_SIZE 32U
#define STAGE_RUNNER_MAIXCAM_WAIT_NOTIFY "WAIT"
#define STAGE_RUNNER_OLED_TEXT_SIZE 24U
#define STAGE_RUNNER_DATA_TEXT_SIZE 24U
#define STAGE_RUNNER_TRACK_MIN_AVG_MMPS 170.0f
#define STAGE_RUNNER_TRACK_LOW_SPEED_DURATION_MS 1000U
#define STAGE_RUNNER_TRACK_DISPLAY_INTERVAL_MS 100U

typedef enum StageRunner_MaixCamResult {
	StageRunnerMaixCamIgnored = 0,
	StageRunnerMaixCamHandled,
	StageRunnerMaixCamContinue,
	StageRunnerMaixCamStop
} StageRunner_MaixCamResult;

// 循迹pid
static PID grayscalePid = {0.1f, 0.0f, 0.0f, 100000.0, 0, 10};

// l1 l2距离
static float distence[2];

// 阶段时间戳
static uint32_t lastStageTime = 0;
// 终点下标
static int Goal = 0;
// 阶段索引
static int StageIndex = 0;
// 阶段标志位
static int StageFlag = 0;

// IMU相关
static IMU_Data_t IMUData;

// 灰度循迹地址
static bool grayscale[GRAYSCALE_SENSOR_CHANNELS];

static const StageCommand *command = NULL;
static int BaseSpeed = 0;
static int RoundSpeed = 0;
static int MaixCamPendingStage = -1;
static int TrackingMonitorStageIndex = -1;
static bool TrackingLowSpeedActive = false;
static uint32_t TrackingLowSpeedStartTime = 0U;
static uint32_t lastTrackingDisplayTime = 0U;
static ArmMotionState armMotionState;

static void StageRunner_ShowTrackingTelemetry(float targetLeft,
										  float targetRight, float irr,
										  float measuredLeft,
										  float measuredRight,
										  float averageSpeed, bool stopped) {
	char line[STAGE_RUNNER_OLED_TEXT_SIZE];

	snprintf(line, sizeof(line), "T:%d,%d", (int)targetLeft,
			 (int)targetRight);
	Display_ShowString(0, 0, line);
	snprintf(line, sizeof(line), "IRR:%.1f", irr);
	Display_ShowString(1, 0, line);
	snprintf(line, sizeof(line), "V:%d,%d", (int)measuredLeft,
			 (int)measuredRight);
	Display_ShowString(2, 0, line);
	snprintf(line, sizeof(line), "%s AVG:%d", stopped ? "STOP" : "RUN",
			 (int)averageSpeed);
	Display_ShowString(3, 0, line);
}

static void StageRunner_SyncTrackingMonitor(uint32_t nowTime) {
	if (TrackingMonitorStageIndex == StageIndex) {
		return;
	}

	TrackingMonitorStageIndex = StageIndex;
	TrackingLowSpeedActive = false;
	TrackingLowSpeedStartTime = 0U;
	lastTrackingDisplayTime =
		nowTime - STAGE_RUNNER_TRACK_DISPLAY_INTERVAL_MS;
}

/*
 * 循迹阶段低速保护。平均速度低于阈值时开始计时，期间任何一次恢复到阈值
 * 或以上都会清零计时；只有连续低速满 1 秒才刹车并锁定故障显示。
 */
static bool StageRunner_HandleTrackingSpeed(uint32_t nowTime, float irr) {
	NewMotor_SpeedCtrl *motor = MotorRuntime_GetController();
	float targetLeft;
	float targetRight;
	float measuredLeft;
	float measuredRight;
	float averageSpeed;
	bool shouldDisplay;

	if (motor == NULL) {
		return false;
	}

	targetLeft = motor->target_left_mmps;
	targetRight = motor->target_right_mmps;
	NewMotorSpeedCtrl_GetMeasuredWheelMmps(motor, &measuredLeft,
										 &measuredRight);
	averageSpeed = NewMotor_LeftRightToLinearSpeedMmps(measuredLeft,
													 measuredRight);

	if (averageSpeed < STAGE_RUNNER_TRACK_MIN_AVG_MMPS) {
		if (!TrackingLowSpeedActive) {
			TrackingLowSpeedActive = true;
			TrackingLowSpeedStartTime = nowTime;
		} else if (getTimeMs(nowTime, TrackingLowSpeedStartTime) >=
				   STAGE_RUNNER_TRACK_LOW_SPEED_DURATION_MS) {
			StageRunner_ShowTrackingTelemetry(
				targetLeft, targetRight, irr, measuredLeft, measuredRight,
				averageSpeed, true);
			MotorRuntime_SetTargetWheelMmps(0, 0);
			MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
			return true;
		}
	} else {
		TrackingLowSpeedActive = false;
		TrackingLowSpeedStartTime = 0U;
	}

	shouldDisplay =
		getTimeMs(nowTime, lastTrackingDisplayTime) >=
		STAGE_RUNNER_TRACK_DISPLAY_INTERVAL_MS;
	if (shouldDisplay) {
		lastTrackingDisplayTime = nowTime;
		StageRunner_ShowTrackingTelemetry(
			targetLeft, targetRight, irr, measuredLeft, measuredRight,
			averageSpeed, false);
	}

	return false;
}

static void StageRunner_SendMaixCamWaitNotify(void) {
	uint8_t notify[STAGE_RUNNER_MAIXCAM_NOTIFY_SIZE];
	uint16_t notifyLength;

	if (MaixCamProtocol_BuildFrame(
			notify, sizeof(notify), MAIXCAM_PROTOCOL_ADDR_CAR,
			MaixCamProtocolType_Data,
			(const uint8_t *)STAGE_RUNNER_MAIXCAM_WAIT_NOTIFY,
			(uint16_t)(sizeof(STAGE_RUNNER_MAIXCAM_WAIT_NOTIFY) - 1U),
			&notifyLength)) {
		MaixCamSerial_SendBytes(notify, notifyLength);
	}
}

static void StageRunner_SendMaixCamStageNotify(const uint8_t *data,
											   uint16_t dataLength) {
	uint8_t notify[STAGE_RUNNER_MAIXCAM_NOTIFY_SIZE];
	uint16_t notifyLength;

	if (MaixCamProtocol_BuildFrame(
			notify, sizeof(notify), MAIXCAM_PROTOCOL_ADDR_EMM,
			MaixCamProtocolType_Stage, data, dataLength,
			&notifyLength)) {
		MaixCamSerial_SendBytes(notify, notifyLength);
	}
}

static void StageRunner_CopyText(char *dest, uint16_t destSize,
								 const uint8_t *src, uint16_t srcLength) {
	uint16_t copyLength;
	uint16_t i;

	if (dest == NULL || destSize == 0U) {
		return;
	}

	copyLength = srcLength;
	if (copyLength >= destSize) {
		copyLength = (uint16_t)(destSize - 1U);
	}

	for (i = 0; i < copyLength; i++) {
		dest[i] = (char)src[i];
	}
	dest[copyLength] = '\0';
}

static StageRunner_MaixCamResult
StageRunner_ExecuteMaixCamFrame(uint8_t *frame, uint16_t length,
								uint32_t nowTime) {
	MaixCamProtocol_Frame packet;

	(void)nowTime;

	if (!MaixCamProtocol_ParseFrame(frame, length, &packet) ||
		packet.addr != MAIXCAM_PROTOCOL_ADDR_CAR) {
		return StageRunnerMaixCamIgnored;
	}

	Display_ShowString(0, 0, "Maix Sending");
	switch (packet.type) {
	case MaixCamProtocolType_Beep:
		Buzzer_Beep();
		return StageRunnerMaixCamHandled;
	case MaixCamProtocolType_Stage:
		if (!MaixCamProtocol_ParseStageData(packet.data, packet.dataLength,
											&MaixCamPendingStage)) {
			return StageRunnerMaixCamIgnored;
		}
		return StageRunnerMaixCamHandled;
	case MaixCamProtocolType_Continue:
		if (MaixCamPendingStage >= 0) {
			StageIndex = MaixCamPendingStage;
		} else {
			StageIndex++;
		}
		MaixCamPendingStage = -1;
		return StageRunnerMaixCamContinue;
	case MaixCamProtocolType_Stop:
		MotorRuntime_SetTargetWheelMmps(0, 0);
		MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
		return StageRunnerMaixCamStop;
	case MaixCamProtocolType_Oled: {
		uint8_t row;
		uint8_t col;
		const uint8_t *text;
		uint16_t textLength;
		char displayText[STAGE_RUNNER_OLED_TEXT_SIZE];

		if (!MaixCamProtocol_ParseOledData(packet.data, packet.dataLength, &row,
										   &col, &text, &textLength)) {
			return StageRunnerMaixCamIgnored;
		}
		StageRunner_CopyText(displayText, sizeof(displayText), text,
							 textLength);
		Display_ShowString(row, col, displayText);
		return StageRunnerMaixCamHandled;
	}
	case MaixCamProtocolType_Data: {
		char dataText[STAGE_RUNNER_DATA_TEXT_SIZE];

		StageRunner_CopyText(dataText, sizeof(dataText), packet.data,
							 packet.dataLength);
		Display_ShowString(2, 0, dataText);
		printf("maix:%s\r\n", dataText);
		return StageRunnerMaixCamHandled;
	}
	default:
		break;
	}

	return StageRunnerMaixCamIgnored;
}

void StageRunner_Init(const StageCommand *commands, int goal,
					  const StageRunner_Config *config) {
	/* 速度配置必须由 main.c 的 stageConfig 显式提供，避免模块内重复默认值。 */
	command = (config != NULL) ? commands : NULL;
	Goal = goal;
	StageIndex = 0;
	StageFlag = 0;
	MaixCamPendingStage = -1;
	TrackingMonitorStageIndex = -1;
	TrackingLowSpeedActive = false;
	TrackingLowSpeedStartTime = 0U;
	lastTrackingDisplayTime = 0U;
	ArmMotionState_Reset(&armMotionState);
	ArmIkMotion_Reset();
	distence[0] = 0.0f;
	distence[1] = 0.0f;
	IMUData.yaw = 0.0f;

	BaseSpeed = (config != NULL) ? config->base_speed_mmps : 0;
	RoundSpeed = (config != NULL) ? config->round_speed_mmps : 0;

	lastStageTime = getNowMs();
}

bool StageRunner_Update(uint32_t nowTime) {
	const StageCommand *stageCommand;
	Stage stage;
	const void *stageData;
	float numData;
	bool shouldStopRun = false;

	if (command == NULL) {
		return false;
	}

	if (getTimeMs(nowTime, lastStageTime) <= STAGE_RUNNER_UPDATE_INTERVAL_MS) {
		return true;
	}

	stageCommand = &command[StageIndex];
	stage = stageCommand->type;
	stageData = stageCommand->data;
	numData = stageCommand->numData;
	StageRunner_SyncTrackingMonitor(nowTime);

	// //-----------灰度模块测试显示----------
	// {
	// 	char temp[24];
	// 	const char *driverName =
	// 		(Grayscale_GetActiveDriver() == GrayscaleDriver12) ? "G12" : "G8";

	// 	Grayscale_Sensor_Read_All(grayscale);
	// 	snprintf(temp, sizeof(temp), "%s:%d %d %d %d %d %d", driverName,
	// 			 grayscale[0], grayscale[1], grayscale[2], grayscale[3],
	// 			 grayscale[4], grayscale[5]);
	// 	Display_ShowString(0, 0, temp);
	// 	snprintf(temp, sizeof(temp), "%d %d %d %d %d %d", grayscale[6],
	// 			 grayscale[7], grayscale[8], grayscale[9], grayscale[10],
	// 			 grayscale[11]);
	// 	Display_ShowString(1, 0, temp);
	// 	snprintf(temp, sizeof(temp), "Read:%d", Grayscale_LastReadOk());
	// 	Display_ShowString(2, 0, temp);
	// }

	switch (stage) {
	case StageEnd: {
		MotorRuntime_SetTargetWheelMmps(0, 0);
		MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
		shouldStopRun = true;
		break;
	}
	case StageRush: {
		// 猛冲一下
		if (StageFlag == 0) {
			MotorRuntime_SetTargetWheelMmps(0.8f * BaseSpeed, 0.8f * BaseSpeed);
			MotorRuntime_ResetDistance();
			StageFlag++;
		}
		if (StageFlag == 1 && (MotorRuntime_GetLeftDistanceMm() > 110 &&
							   MotorRuntime_GetRightDistanceMm() > 110)) {
			MotorRuntime_SetTargetWheelMmps(0, 0);
			MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
			StageFlag++;
		} else if (StageFlag == 1) {
			float speedP =
				0.8f * (1 - MotorRuntime_GetLeftDistanceMm() / 110.0f) + 0.2f;
			MotorRuntime_SetTargetWheelMmps(0.8f * speedP * BaseSpeed,
											0.8f * speedP * BaseSpeed);
		}
		if (StageFlag <= 10 && StageFlag >= 2) {

			StageFlag++;
		}
		if (StageFlag >= 11) {
			StageFlag = 0;
			StageIndex++;
		}
		break;
	}
	case StageRight: {
		// StageRight
		if (StageFlag == 0) {
			MotorRuntime_SetTargetWheelMmps(BaseSpeed, BaseSpeed);
			Grayscale_Zero(grayscale);
			StageFlag++;
		}
		if (!Grayscale_Cross(grayscale, 1)) {
			grayscalePid.t = getTimeMs(nowTime, lastStageTime);
			float irr = Grayscale_Line(&grayscalePid, grayscale);
			MotorRuntime_SetTargetRobot(BaseSpeed, irr);
			if (StageRunner_HandleTrackingSpeed(nowTime, irr)) {
				shouldStopRun = true;
			}

		} else {
			StageFlag = 0;
			StageIndex++;
		}
		break;
	}
	case StageRightRound: {
		// StageRightRound
		bool reached;
		float base;

		if (StageFlag == 0) {
			StageFlag++;
		}
		(void)Grayscale_GetTurnControl(GrayscaleTurnRight, &reached, &base);
		if (reached) {
			MotorRuntime_SetTargetWheelMmps(0, 0);
			MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
			StageFlag = 0;
			lastStageTime = nowTime;
			StageIndex++;
		} else {
			MotorRuntime_SetTargetWheelMmps((base * RoundSpeed),
										-(base * RoundSpeed));
		}
		break;
	}
	case StageLeft: {
		// StageLeft
		if (StageFlag == 0) {
			MotorRuntime_SetTargetWheelMmps(BaseSpeed, BaseSpeed);
			Grayscale_Zero(grayscale);
			StageFlag++;
		}
		if (!Grayscale_Cross(grayscale, 2)) {
			grayscalePid.t = getTimeMs(nowTime, lastStageTime);
			float irr = Grayscale_Line(&grayscalePid, grayscale);
			MotorRuntime_SetTargetRobot(BaseSpeed, irr);
			if (StageRunner_HandleTrackingSpeed(nowTime, irr)) {
				shouldStopRun = true;
			}

		} else {
			StageFlag = 0;
			StageIndex++;
		}
		break;
	}
	case StageLeftRound: {
		// StageLeftRound
		bool reached;
		float base;

		if (StageFlag == 0) {
			StageFlag++;
		}
		(void)Grayscale_GetTurnControl(GrayscaleTurnLeft, &reached, &base);
		if (reached) {
			MotorRuntime_SetTargetWheelMmps(0, 0);
			MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
			StageFlag = 0;
			lastStageTime = nowTime;
			StageIndex++;
		} else {
			MotorRuntime_SetTargetWheelMmps(-(base * RoundSpeed),
										(base * RoundSpeed));
		}
		break;
	}
	case StageCross: {
		// StageCross
		if (StageFlag == 0) {
			MotorRuntime_SetTargetWheelMmps(BaseSpeed, BaseSpeed);
			MotorRuntime_ResetDistance();
			StageFlag++;
		}
		if (StageFlag > 0 && StageFlag <= 5) {
			grayscalePid.t = getTimeMs(nowTime, lastStageTime);
			float irr = Grayscale_Line(&grayscalePid, grayscale);
			MotorRuntime_SetTargetRobot(BaseSpeed, irr);
			if (StageRunner_HandleTrackingSpeed(nowTime, irr)) {
				shouldStopRun = true;
				break;
			}
		}
		if (StageFlag > 5) {
			StageFlag = 0;
			StageIndex++;
		}
		if ((Grayscale_Cross(grayscale, 0) || Grayscale_Cross(grayscale, 2) ||
			 Grayscale_Cross(grayscale, 1)) &&
			StageFlag % 2 == 1) {

			if (StageFlag == 3) {
				distence[0] = MotorRuntime_GetLeftDistanceMm() - 18;
			}
			if (StageFlag == 5) {
				distence[1] = MotorRuntime_GetLeftDistanceMm() - 18;
			}
			MotorRuntime_ResetDistance();
			StageFlag++;
		}
		if (!(Grayscale_Cross(grayscale, 0) || Grayscale_Cross(grayscale, 2) ||
			  Grayscale_Cross(grayscale, 1)) &&
			StageFlag % 2 == 0) {
			StageFlag++;
		}
		break;
	}
	case StageStartJudge: {
		// StageStartJudge
		if (Grayscale_Cross(grayscale, 1)) {
			// 右直角, AD起点
			StageFlag = 0;
			StageIndex = 23;

		} else if (Grayscale_Cross(grayscale, 2)) {
			// 左直角, AB起点
			StageFlag = 0;
			StageIndex++;
		} else {
		}
		break;
	}
	case StageFinsih: {
		// StageFinsih
		grayscalePid.t = getTimeMs(nowTime, lastStageTime);
		float irr = Grayscale_Line(&grayscalePid, grayscale);
		irr = 0.0f;
		MotorRuntime_SetTargetRobot(0.5f * BaseSpeed, irr);
		if (Grayscale_OnlineNum(grayscale) > 0) {
			MotorRuntime_SetTargetWheelMmps(0, 0);
			MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
			Buzzer_Beep();
			shouldStopRun = true;
		}
		break;
	}
	case StageBizz: {
		// StageBizz
		MotorRuntime_SetTargetWheelMmps(0, 0);
		MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
		Buzzer_Beep();
		StageIndex++;
		break;
	}
	case StageFake: {
		// StageFake
		if (Grayscale_OnlineNum(grayscale) > 0) {
			StageIndex -= 2;
		} else {
			StageIndex++;
		}
		break;
	}
	case StageStop: {
		// StageStop
		// if (StageFlag == 0) {
		// 	NewMotorSpeedCtrl_SetTargetWheelMmps(
		// 		&motor, 0.5 * -BaseSpeed, 0.5 * -BaseSpeed);
		// 	rightDistance = 0;
		// 	leftDistance = 0;
		// 	StageFlag++;
		// }
		// if (StageFlag == 1 &&
		// 	(NewMotor_EncoderDeltaToDistanceMm(leftDistance) < -80 &&
		// 	 NewMotor_EncoderDeltaToDistanceMm(rightDistance) < -80)) {
		// 	NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
		// 	NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
		// 	StageFlag++;
		// }

		// grayscalePid.t = getTimeMs(nowTime, lastStageTime);
		// Grayscale_Line(&grayscalePid, grayscale);
		// NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0.5 *
		// -BaseSpeed, 									 0.5 *
		// -BaseSpeed); if (grayscale[0] == 0 && grayscale[1] == 0 &&
		// grayscale[2] == 0 && grayscale[3] == 0
		// && 	grayscale[4] == 0 && grayscale[5] == 0 && 	grayscale[6]
		// == 0 && grayscale[7] == 0 && StageFlag == 0) { 	StageFlag++;
		// }
		// if (!(grayscale[0] == 0 && grayscale[1] == 0 &&
		// 	  grayscale[2] == 0 && grayscale[3] == 0 &&
		// 	  grayscale[4] == 0 && grayscale[5] == 0 &&
		// 	  grayscale[6] == 0 && grayscale[7] == 0) &&
		// 	StageFlag == 1) {
		// 	NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
		// 	NewMotor_Stop(NEWMOTOR_STOP_COAST);
		// 	buzzer_beep();
		// 	StageFlag = 0;
		// 	shouldStopRun = true;
		// }
		MotorRuntime_SetTargetWheelMmps(0, 0);
		MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
		break;
	}
	case StageTurn: {
		static int isRight = -1;
		if (StageFlag == 0) {
			if (numData < 0.0f) {
				isRight = 1;
			} else {
				isRight = -1;
			}
			IMUData.yaw = 0.0f;
			IMU_ZeroYaw();
			StageFlag++;
		}
		MotorRuntime_SetTargetWheelMmps(-isRight * (RoundSpeed),
										isRight * (RoundSpeed));
		(void)IMU_ReadAll(&IMUData);
		if (IMUData.yaw > (float)(numData - 0.3f) &&
			IMUData.yaw < (float)(numData + 0.3f)) {

			MotorRuntime_SetTargetWheelMmps(0, 0);
			MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
			StageFlag = 0;
			lastStageTime = nowTime;
			StageIndex++;
		} else {
			float error =
				fminf((-IMUData.yaw + numData) / numData + 0.3f, 1.0f);
			char temp[21];
			sprintf(temp, "angel:%.2f", IMUData.yaw);
			Display_ShowString(3, 0, temp);
			MotorRuntime_SetTargetWheelMmps(-isRight * (error * RoundSpeed),
											isRight * (error * RoundSpeed));
		}
		break;
	}
	case StageSkip: {
		// StageSkip
		int offset = (3 - Goal) * 3;
		StageIndex += offset + 1;
		break;
	}
	case StageForward: {
		static int32_t stageSpeed = 0;
		static float stageDistence = 0.0f;
		float avgDistance;
		if (StageFlag == 0) {
			MotorRuntime_ResetDistance();
			if (stageData == NULL) {
				stageSpeed = BaseSpeed;
				stageDistence = numData;
			} else {
				stageSpeed = ((const StageForwardData *)stageData)->speed;
				stageDistence = ((const StageForwardData *)stageData)->length;
			}
			IMUData.yaw = 0.0f;
			IMU_ZeroYaw();
			StageFlag++;
		} else {
			avgDistance = MotorRuntime_GetAverageDistanceMm();
			if (fabs(stageDistence - avgDistance) < stageSpeed / 50.0f) {
				MotorRuntime_SetTargetWheelMmps(0, 0);
				MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
				StageFlag = 0;
				StageIndex++;
			} else if (fabs(stageDistence - avgDistance) < stageSpeed / 4.0f) {
				MotorRuntime_SetTargetWheelMmps(
					fminf(
						fabs(stageDistence - MotorRuntime_GetLeftDistanceMm()) /
								(stageSpeed / 4.0f) +
							0.2f,
						1.0f) *
						stageSpeed,
					fminf(
						fabs(stageDistence - MotorRuntime_GetLeftDistanceMm()) /
								(stageSpeed / 4.0f) +
							0.2f,
						1.0f) *
						stageSpeed);
			} else {
				IMU_ReadAll(&IMUData);

				MotorRuntime_SetTargetWheelMmps(stageSpeed - 20 * IMUData.yaw,
												stageSpeed + 20 * IMUData.yaw);
			}
			char temp[21];
		}
		break;
	}
	case StageMaixCamCommand: {
		uint8_t frame[STAGE_RUNNER_MAIXCAM_FRAME_SIZE];
		uint16_t frameLength;
		StageRunner_MaixCamResult maixResult;
		const StageMaixCamCommandData *maixData =
			(const StageMaixCamCommandData *)stageData;

		if (StageFlag == 0) {
			MotorRuntime_SetTargetWheelMmps(0, 0);
			MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
			MaixCamPendingStage = -1;
			// 获取数据数组
			if (maixData != NULL) {
				StageRunner_SendMaixCamStageNotify(maixData->bytes,
												   maixData->length);
			}
			Display_ShowString(0, 0, "Wait Maix");
			StageFlag = 1;
		}

		if (MaixCamSerial_TryReadFrame(frame, sizeof(frame), &frameLength)) {
			maixResult =
				StageRunner_ExecuteMaixCamFrame(frame, frameLength, nowTime);
			if (maixResult == StageRunnerMaixCamContinue) {
				Display_ShowString(0, 0, "Maix Send Conti");
				StageFlag = 0;
				lastStageTime = nowTime;
			} else if (maixResult == StageRunnerMaixCamStop) {
				Display_ShowString(0, 0, "Maix Send Stop");
				StageFlag = 0;
				MaixCamPendingStage = -1;
				shouldStopRun = true;
			}
		}
		break;
	}
	case StageButtonContinue: {
		ButtonSelect_Event event;

		/*
		 * 此阶段用于需要人工确认后才继续的路线段。进入阶段时先刹停并
		 * 丢弃之前阶段遗留的按键事件，避免刚进入就被旧事件跳过。
		 */
		if (StageFlag == 0) {
			MotorRuntime_SetTargetWheelMmps(0, 0);
			MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
			ButtonSelect_ResetEvents();
			Display_ShowString(0, 0, "Press B2 Continue");
			StageFlag = 1;
		}

		/* B1 事件在此阶段被消费但不改变流程；仅 B2（Start）可继续。 */
		while ((event = ButtonSelect_TakeEvent()) != ButtonSelectEventNone) {
			if (event == ButtonSelectEventStart) {
				Display_ShowString(0, 0, "Continue...");
				StageFlag = 0;
				StageIndex++;
				lastStageTime = nowTime;
				break;
			}
		}
		break;
	}
	case StageArmMotion: {
		const StageArmMotionData *armData =
			(const StageArmMotionData *)stageData;
		ArmMotionState_Status armStatus;

		if (StageFlag == 0) {
			/* 机械臂动作期间保持小车制动，防止移动造成机械臂碰撞。 */
			MotorRuntime_SetTargetWheelMmps(0, 0);
			MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
			if ((armData == NULL) ||
				!ArmMotionState_Start(&armMotionState, &armData->sequence,
									  nowTime)) {
				Display_ShowString(0, 0, "ARM MOTION FAIL");
				shouldStopRun = true;
				break;
			}
			StageFlag = 1;
		}

		armStatus = ArmMotionState_Update(&armMotionState, nowTime);
		if (armStatus == ArmMotionStateCompleted) {
			StageFlag = 0;
			StageIndex++;
		} else if (armStatus == ArmMotionStateFailed) {
			Display_ShowString(0, 0, "ARM MOTION FAIL");
			shouldStopRun = true;
		}
		break;
	}
	case StageArmIkMotion: {
		const StageArmIkMotionData *armData =
			(const StageArmIkMotionData *)stageData;
		ArmMotionState_Status armStatus;

		if (StageFlag == 0) {
			/* 笛卡尔动作同样必须在车辆静止时执行。 */
			MotorRuntime_SetTargetWheelMmps(0, 0);
			MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
			if ((armData == NULL) ||
				!ArmIkMotion_Start(armData->yawDeg, armData->xMm,
							  armData->yMm)) {
				Display_ShowString(0, 0,
					ArmIkMotion_LastStartWasReachabilityFailure() ?
						"CANT REACH" : "ARM IK FAIL");
				shouldStopRun = true;
				break;
			}
			StageFlag = 1;
		}

		armStatus = ArmIkMotion_Update();
		if (armStatus == ArmMotionStateCompleted) {
			StageFlag = 0;
			StageIndex++;
		} else if (armStatus == ArmMotionStateFailed) {
			Display_ShowString(0, 0, "ARM IK FAIL");
			shouldStopRun = true;
		}
		break;
	}
	default:
		break;
	}

	lastStageTime = nowTime;

	return !shouldStopRun;
}

int StageRunner_GetStageIndex(void) { return StageIndex; }

int StageRunner_GetStageFlag(void) { return StageFlag; }
