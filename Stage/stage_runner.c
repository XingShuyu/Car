#include "Stage/stage_runner.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "BasicMicroLib/PID.h"
#include "BasicMicroLib/getTime.h"
#include "Communication/maixcam_protocol.h"
#include "Communication/maixcam_serial.h"
#include "Drivers/buzzer.h"
#include "GrayScale/Grayscale_Scan.h"
#include "GrayScale/grayscale_sensor.h"
#include "IMU/imu.h"
#include "Motor/motor_runtime.h"
#include "OLED/display.h"

#define STAGE_RUNNER_DEFAULT_BASE_SPEED_MMPS 300
#define STAGE_RUNNER_DEFAULT_ROUND_SPEED_MMPS 150
#define STAGE_RUNNER_UPDATE_INTERVAL_MS 10U
#define STAGE_RUNNER_MAIXCAM_FRAME_SIZE 32U
#define STAGE_RUNNER_MAIXCAM_NOTIFY_SIZE 32U
#define STAGE_RUNNER_MAIXCAM_WAIT_NOTIFY "WAIT"
#define STAGE_RUNNER_OLED_TEXT_SIZE 24U
#define STAGE_RUNNER_DATA_TEXT_SIZE 24U

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
static bool grayscale[8];

static const StageCommand *command = NULL;
static int BaseSpeed = STAGE_RUNNER_DEFAULT_BASE_SPEED_MMPS;
static int RoundSpeed = STAGE_RUNNER_DEFAULT_ROUND_SPEED_MMPS;
static int MaixCamPendingStage = -1;

static void StageRunner_SendMaixCamWaitNotify(void)
{
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
											   uint16_t dataLength)
{
	uint8_t notify[STAGE_RUNNER_MAIXCAM_NOTIFY_SIZE];
	uint16_t notifyLength;

	if (MaixCamProtocol_BuildFrame(
			notify, sizeof(notify), MAIXCAM_PROTOCOL_ADDR_CAR,
			MaixCamProtocolType_Stage, data, dataLength,
			&notifyLength)) {
		MaixCamSerial_SendBytes(notify, notifyLength);
	}
}

static void StageRunner_CopyText(char *dest, uint16_t destSize,
								 const uint8_t *src, uint16_t srcLength)
{
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

static StageRunner_MaixCamResult StageRunner_ExecuteMaixCamFrame(uint8_t *frame,
																 uint16_t length,
																 uint32_t nowTime)
{
	MaixCamProtocol_Frame packet;

	(void)nowTime;

	if (!MaixCamProtocol_ParseFrame(frame, length, &packet) ||
		packet.addr != MAIXCAM_PROTOCOL_ADDR_CAR) {
		return StageRunnerMaixCamIgnored;
	}

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

		if (!MaixCamProtocol_ParseOledData(packet.data, packet.dataLength,
										   &row, &col, &text, &textLength)) {
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
		printf("maix:%s\r\n", dataText);
		return StageRunnerMaixCamHandled;
	}
	default:
		break;
	}

	return StageRunnerMaixCamIgnored;
}

void StageRunner_Init(const StageCommand *commands, int goal,
					  const StageRunner_Config *config)
{
	command = commands;
	Goal = goal;
	StageIndex = 0;
	StageFlag = 0;
	MaixCamPendingStage = -1;
	distence[0] = 0.0f;
	distence[1] = 0.0f;
	IMUData.yaw = 0.0f;

	if (config != NULL) {
		BaseSpeed = config->base_speed_mmps;
		RoundSpeed = config->round_speed_mmps;
	} else {
		BaseSpeed = STAGE_RUNNER_DEFAULT_BASE_SPEED_MMPS;
		RoundSpeed = STAGE_RUNNER_DEFAULT_ROUND_SPEED_MMPS;
	}

	lastStageTime = getNowMs();
}

bool StageRunner_Update(uint32_t nowTime)
{
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
			MotorRuntime_SetTargetWheelMmps(0.8f * BaseSpeed,
											0.8f * BaseSpeed);
			MotorRuntime_ResetDistance();
			StageFlag++;
		}
		if (StageFlag == 1 &&
			(MotorRuntime_GetLeftDistanceMm() > 110 &&
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

		} else {
			StageFlag = 0;
			StageIndex++;
		}
		break;
	}
	case StageRightRound: {
		// StageRightRound
		float base = 1.0f;
		if (StageFlag == 0) {
			StageFlag++;
		}
		if (_read_channel_stable(4)) {
			MotorRuntime_SetTargetWheelMmps(0, 0);
			MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
			StageFlag = 0;
			lastStageTime = nowTime;
			StageIndex++;
		} else if (_read_channel_stable(7)) {
			base = 0.5f;
		} else if (_read_channel_stable(6)) {
			base = 0.3f;
		} else if (_read_channel_stable(5)) {
			base = 0.1f;
		}
		MotorRuntime_SetTargetWheelMmps((base * RoundSpeed),
										-(base * RoundSpeed));
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

		} else {
			StageFlag = 0;
			StageIndex++;
		}
		break;
	}
	case StageLeftRound: {
		// StageLeftRound
		float base = 1.0f;
		if (StageFlag == 0) {
			StageFlag++;
		}
		if (_read_channel_stable(3)) {
			MotorRuntime_SetTargetWheelMmps(0, 0);
			MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
			StageFlag = 0;
			lastStageTime = nowTime;
			StageIndex++;
		} else if (_read_channel_stable(0)) {
			base = 0.5f;
		} else if (_read_channel_stable(1)) {
			base = 0.3f;
		} else if (_read_channel_stable(2)) {
			base = 0.1f;
		}
		MotorRuntime_SetTargetWheelMmps(-(base * RoundSpeed),
										(base * RoundSpeed));
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
		}
		if (StageFlag > 5) {
			StageFlag = 0;
			printf("l1: %.2f", distence[0]);
			printf("l2: %.2f", distence[1]);
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
		if (!(grayscale[0] == 0 && grayscale[1] == 0 && grayscale[2] == 0 &&
			  grayscale[3] == 0 && grayscale[4] == 0 && grayscale[5] == 0 &&
			  grayscale[6] == 0 && grayscale[7] == 0)) {
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
		if (_read_channel_stable(0) || _read_channel_stable(1) ||
			_read_channel_stable(2) || _read_channel_stable(3) ||
			_read_channel_stable(4) || _read_channel_stable(5) ||
			_read_channel_stable(6) || _read_channel_stable(7)) {
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
		// NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
		// NewMotor_Stop(NEWMOTOR_STOP_COAST);
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
			float error = fminf((-IMUData.yaw + numData) / numData + 0.2f, 1.0f);
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
					fminf(fabs(stageDistence - MotorRuntime_GetLeftDistanceMm()) /
								  (stageSpeed / 4.0f) +
							  0.2f,
						  1.0f) *
						stageSpeed,
					fminf(fabs(stageDistence - MotorRuntime_GetLeftDistanceMm()) /
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
			sprintf(temp, "len:%.2f", fabs(stageDistence - avgDistance));
			Display_ShowString(2, 0, temp);
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
			//获取数据数组
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
				StageFlag = 0;
				lastStageTime = nowTime;
			} else if (maixResult == StageRunnerMaixCamStop) {
				StageFlag = 0;
				MaixCamPendingStage = -1;
				shouldStopRun = true;
			}
		}
		break;
	}
	default:
		break;
	}

	lastStageTime = nowTime;

	return !shouldStopRun;
}

int StageRunner_GetStageIndex(void)
{
	return StageIndex;
}

int StageRunner_GetStageFlag(void)
{
	return StageFlag;
}
