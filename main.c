#include "BasicMicroLib/delay.h"
#include "BasicMicroLib/getTime.h"
#include "BasicMicroLib/usart.h"
#include "Emm/Emm.h"
#include "GrayScale/Grayscale_Scan.h"
#include "IMU/imu.h"
#include "Motor/newmotor_speed_ctrl.h"
#include "OLED/display.h"
#include "Stage.h"
#include "ti_msp_dl_config.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#define MOTOR_STEP_TEST_ENABLE 0
#define MOTOR_STEP_TEST_TARGET_MMPS 500.0f
#define MOTOR_STEP_TEST_CONTROL_PERIOD_US 500U
#define MOTOR_STEP_TEST_MEASURE_WINDOW_US 1000U
#define MOTOR_STEP_TEST_TIMEOUT_MS 5000U
#define MOTOR_STEP_TEST_REACHED_RATIO 0.95f
#define MOTOR_STEP_TEST_ABS_TOL_MMPS 25.0f
#define MOTOR_STEP_TEST_STABLE_TIME_MS 500U
#define MOTOR_STEP_TEST_LOG_INTERVAL_MS 100U
#define MOTOR_STEP_TEST_PWM_SAT_TICKS 2000

// 循迹pid
PID grayscalePid = {0.1f, 0.0f, 0.0f, 100000.0, 0, 10};

// 基础速度 
int BaseSpeed = 300;
int RoundSpeed = 150;
// l1 l2距离
float distence[2];
//-------------------
// 各种时间声明
// 获取电机速度时间戳
uint32_t lastMotorSpeedTime = 0;
// 数据输出时间戳
uint32_t lastUartTime = 0;
// 阶段时间戳
uint32_t lastStageTime = 0;
// 状态机下标
int TextIndex = 0;
// 终点下标
int Goal = 0;
// 阶段索引
int StageIndex = 0;
// 阶段标志位
int StageFlag = 0;
// maixcam串口相关
volatile uint8_t maixcam_buff[32] = {0};
volatile uint16_t maixcam_length = 0;
volatile uint8_t maixcam_flag = 0;
// 串口相关
volatile uint8_t recv0_buff[128] = {0};
volatile uint16_t recv0_length = 0;
volatile uint8_t recv0_flag = 0;

// IMU相关
IMU_Data_t IMUData;

// 灰度循迹地址
bool grayscale[8];

// volatile float motorRightSpeed = 0; //	速度(m/s)
// volatile float motorLeftSpeed = 0;	//	速度(m/s)
NewMotor_SpeedCtrl motor;
volatile int32_t motorLeftCount = 0;
volatile int32_t motorRightCount = 0;
int leftDistance, rightDistance;

void buzzer_beep(void);
static void BluetoothSerial_OnRxByte(uint8_t receivedData);
#if MOTOR_STEP_TEST_ENABLE
static void MotorClosedLoopStepTest(void);
static bool MotorStepTest_IsReached(float measured_mmps, float target_mmps);
#endif

int main(void) {
	//--------------------------------------
	//                初始化
	//--------------------------------------
	SYSCFG_DL_init(); // 由SysConfig自动生成的初始化函数
	//---------------中断使能----------------

	// 开启 GPIOA 和 GPIOB 的全局中断 (因为编码器引脚跨越了这两个端口)
	NVIC_EnableIRQ(MotorMonitor_GPIOA_INT_IRQN);
	NVIC_EnableIRQ(GPIOB_INT_IRQn);
	USART_SetRxByteCallback(BluetoothSerial_OnRxByte);
	USART_Init(); // 使能UART中断（接收依赖此步骤）
	setvbuf(stdout, NULL, _IONBF, 0);

	Display_Init(); // 初始化显示屏

	NVIC_EnableIRQ(UART_MAIXCAM_INST_INT_IRQN); // 初始化maixcam

	TimeBase_Init(); // 初始化计时器

	// 使能云台
	Emm_Init(1);
	delay_ms(10);
	Emm_Init(2);
	delay_ms(10);

	// 电机初始化
	DL_TimerG_startCounter(MotorLeft_INST);
	DL_TimerG_startCounter(MotorRight_INST);
	NewMotorSpeedCtrl_Init(&motor, 0.001f);
	NewMotorSpeedCtrl_SetPid(&motor, 1.0,30.0, 0.0);
	NewMotorSpeedCtrl_SetOutputLimit(&motor, -2000, 2000);
	//NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, BaseSpeed, BaseSpeed);
	printf("OK");

#if MOTOR_STEP_TEST_ENABLE
	MotorClosedLoopStepTest();
#endif

	// IMU初始化
	{
		int temp = 0;
		temp = IMU_Init();
		if (temp) {
			if (temp == 1) {
				Display_ShowString(0, 0, "HWT101 Ready");
			} else if (temp == 2) {
				Display_ShowString(0, 0, "JY901S Ready");
			} else if (temp == 3) {
				Display_ShowString(0, 0, "MPU6050 Ready");
			}
		} else {
			Display_ShowString(0, 0, "IMU Faid");
		}
		delay_ms(1000);
	}

	// 获取启动时间tick
	Display_ShowString(0, 0, "Car Ready"); // 可选：开机显示欢迎信息
	delay_ms(2000);
	Display_Clear();
	IMU_ZeroYaw();
	startTime = getNowMs();
	// int TempIndex = 0;
	TextIndex = 0;
	while (getTimeMs(getNowMs(), startTime) < 1000) {
		char str[8];
		sprintf(str, "set: %d", TextIndex);
		Display_ShowString(0, 0, str);
	}
	const StageCommand *command = commandList[TextIndex];
	if (TextIndex == 3) {
		TextIndex = 0;
		startTime = getNowMs();
		while (getTimeMs(getNowMs(), startTime) < 3000) {
			char str[8];
			sprintf(str, "goal: %d", TextIndex);
			Display_ShowString(0, 0, str);
		}
		Goal = TextIndex;
	}
	startTime = getNowMs();
	buzzer_beep();
	lastStageTime = getNowMs();
	NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, BaseSpeed, BaseSpeed);

	while (1) {
		// 更新当前时间
		nowTime = getNowMs();
		USART_PollTx();
		// 每30ms获取电机运行圈数
		uint32_t nowUs = getNowUs();
		if (getTimeUs(nowUs, lastMotorSpeedTime) > 30000) {
			int32_t leftCountSnapshot;
			int32_t rightCountSnapshot;
			motor.sample_period_s =
				(float)getTimeUs(nowUs, lastMotorSpeedTime) / 1000000.0f;
			lastMotorSpeedTime = nowUs;

			// 原子化读取并清零编码器计数，避免与中断并发导致丢脉冲
			__disable_irq();
			leftCountSnapshot = motorLeftCount;
			rightCountSnapshot = motorRightCount;
			motorLeftCount = 0;
			motorRightCount = 0;
			__enable_irq();
			leftDistance += leftCountSnapshot;
			rightDistance += rightCountSnapshot;

			int sped =
				(int)(NewMotor_EncoderDeltaToDistanceMm(leftCountSnapshot) /
					  motor.sample_period_s);
			printf("Back:%d\n", sped);
			if (getTimeMs(nowTime, lastUartTime) >=
				MOTOR_STEP_TEST_LOG_INTERVAL_MS) {
				lastUartTime = nowTime;
				
			}

			NewMotorSpeedCtrl_UpdateByEncoderDelta(&motor, leftCountSnapshot,
												   rightCountSnapshot);
		}

		if (getTimeMs(nowTime, lastStageTime) > 5) {
			const StageCommand *stageCommand = &command[StageIndex];
			Stage stage = stageCommand->type;
			const void *stageData = stageCommand->data;
			float numData = stageCommand->numData;
			bool shouldStopRun = false;
			(void)stageData;

			switch (stage) {
			case StageEnd: {
				NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
				NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
				shouldStopRun = true;
				break;
			}
			case StageRush: {
				// 猛冲一下
				if (StageFlag == 0) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(
						&motor, 0.8 * BaseSpeed, 0.8 * BaseSpeed);
					rightDistance = 0;
					leftDistance = 0;
					StageFlag++;
				}
				if (StageFlag == 1 &&
					(NewMotor_EncoderDeltaToDistanceMm(leftDistance) > 110 &&
					 NewMotor_EncoderDeltaToDistanceMm(rightDistance) > 110)) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
					NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
					StageFlag++;
				} else if (StageFlag == 1) {
					float speedP = 0.8 * (1 - NewMotor_EncoderDeltaToDistanceMm(
												  leftDistance) /
												  110.0) +
								   0.2;
					NewMotorSpeedCtrl_SetTargetWheelMmps(
						&motor, 0.8 * speedP * BaseSpeed,
						0.8 * speedP * BaseSpeed);
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
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, BaseSpeed,
														 BaseSpeed);
					Grayscale_Zero(grayscale);
					StageFlag++;
				}
				if (!Grayscale_Cross(grayscale, 1)) {
					grayscalePid.t = getTimeMs(nowTime, lastStageTime);
					float irr = Grayscale_Line(&grayscalePid, grayscale);
					NewMotorSpeedCtrl_SetTargetRobot(&motor, BaseSpeed, irr);

				} else {
					StageFlag = 0;
					StageIndex++;
				}
				break;
			}
			case StageRightRound: {
				// StageRightRound
				float base = 1.0;
				if (StageFlag == 0) {
					StageFlag++;
				}
				if (_read_channel_stable(4)) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
					NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
					StageFlag = 0;
					lastStageTime = nowTime;
					StageIndex++;
				} else if (_read_channel_stable(7)) {
					base = 0.5;
				} else if (_read_channel_stable(6)) {
					base = 0.3;
				} else if (_read_channel_stable(5)) {
					base = 0.1;
				}
				NewMotorSpeedCtrl_SetTargetWheelMmps(
					&motor, (base * RoundSpeed), -(base * RoundSpeed));
				break;
			}
			case StageLeft: {
				// StageLeft
				if (StageFlag == 0) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, BaseSpeed,
														 BaseSpeed);
					Grayscale_Zero(grayscale);
					StageFlag++;
				}
				if (!Grayscale_Cross(grayscale, 2)) {
					grayscalePid.t = getTimeMs(nowTime, lastStageTime);
					float irr = Grayscale_Line(&grayscalePid, grayscale);
					NewMotorSpeedCtrl_SetTargetRobot(&motor, BaseSpeed, irr);

				} else {
					StageFlag = 0;
					StageIndex++;
				}
				break;
			}
			case StageLeftRound: {
				// StageLeftRound
				float base = 1.0;
				if (StageFlag == 0) {
					StageFlag++;
				}
				if (_read_channel_stable(3)) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
					NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
					StageFlag = 0;
					lastStageTime = nowTime;
					StageIndex++;
				} else if (_read_channel_stable(0)) {
					base = 0.5;
				} else if (_read_channel_stable(1)) {
					base = 0.3;
				} else if (_read_channel_stable(2)) {
					base = 0.1;
				}
				NewMotorSpeedCtrl_SetTargetWheelMmps(
					&motor, -(base * RoundSpeed), (base * RoundSpeed));
				break;
			}
			case StageCross: {
				// StageCross
				if (StageFlag == 0) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, BaseSpeed,
														 BaseSpeed);
					rightDistance = 0;
					leftDistance = 0;
					StageFlag++;
				}
				if (StageFlag > 0 && StageFlag <= 5) {
					grayscalePid.t = getTimeMs(nowTime, lastStageTime);
					float irr = Grayscale_Line(&grayscalePid, grayscale);
					NewMotorSpeedCtrl_SetTargetRobot(&motor, BaseSpeed, irr);
				}
				if (StageFlag > 5) {
					StageFlag = 0;
					printf("l1: %.2f", distence[0]);
					printf("l2: %.2f", distence[1]);
					StageIndex++;
				}
				if ((Grayscale_Cross(grayscale, 0) ||
					 Grayscale_Cross(grayscale, 2) ||
					 Grayscale_Cross(grayscale, 1)) &&
					StageFlag % 2 == 1) {

					if (StageFlag == 3) {
						distence[0] =
							NewMotor_EncoderDeltaToDistanceMm(leftDistance) -
							18;
					}
					if (StageFlag == 5) {
						distence[1] =
							NewMotor_EncoderDeltaToDistanceMm(leftDistance) -
							18;
					}
					rightDistance = 0;
					leftDistance = 0;
					StageFlag++;
				}
				if (!(Grayscale_Cross(grayscale, 0) ||
					  Grayscale_Cross(grayscale, 2) ||
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
				irr = 0.0;
				NewMotorSpeedCtrl_SetTargetRobot(&motor, 0.5 * BaseSpeed, irr);
				if (!(grayscale[0] == 0 && grayscale[1] == 0 &&
					  grayscale[2] == 0 && grayscale[3] == 0 &&
					  grayscale[4] == 0 && grayscale[5] == 0 &&
					  grayscale[6] == 0 && grayscale[7] == 0)) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
					NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
					buzzer_beep();
					shouldStopRun = true;
				}
				break;
			}
			case StageBizz: {
				// StageBizz
				NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
				NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
				buzzer_beep();
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
				// NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0.5 * -BaseSpeed,
				// 									 0.5 * -BaseSpeed);
				// if (grayscale[0] == 0 && grayscale[1] == 0 &&
				// 	grayscale[2] == 0 && grayscale[3] == 0 &&
				// 	grayscale[4] == 0 && grayscale[5] == 0 &&
				// 	grayscale[6] == 0 && grayscale[7] == 0 && StageFlag == 0) {
				// 	StageFlag++;
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
				NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
				NewMotor_Stop(NEWMOTOR_STOP_COAST);
				break;
			}
			case StageTurn: {
				static int isRight = -1;
				if (StageFlag == 0) {
					if (numData<0.0) {
						isRight = 1;
					}
					else {
						isRight = -1;
					}
					IMUData.yaw = 0.0;
					IMU_ZeroYaw();
					StageFlag++;
				}
				NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, -isRight*(RoundSpeed),
													 isRight*(RoundSpeed));
				(void)IMU_ReadAll(&IMUData);
				if (IMUData.yaw > (float)(numData-0.5) && IMUData.yaw < (float)(numData+0.5)) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
					NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
					StageFlag = 0;
					lastStageTime = nowTime;
					StageIndex++;
				} else {
					float error = (-IMUData.yaw + numData) / numData + 0.1;
					NewMotorSpeedCtrl_SetTargetWheelMmps(
						&motor, -isRight*(error * RoundSpeed), isRight*(error * RoundSpeed));
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
				static float stageDistence = 0.0;
				float avgDistance;
				if (StageFlag == 0) {
					leftDistance = 0;
					rightDistance = 0;
					if (stageData == NULL) {
						stageSpeed = BaseSpeed;
						stageDistence = numData;
					}
					else {
						stageSpeed = ((const StageForwardData*)stageData)->speed;
						stageDistence = ((const StageForwardData*)stageData)->length;
					}
					NewMotorSpeedCtrl_Reset(&motor);
					StageFlag++;
				}
				else {
					avgDistance = 0.5*(NewMotor_EncoderDeltaToDistanceMm(leftDistance)+NewMotor_EncoderDeltaToDistanceMm(rightDistance));
					if (fabs(stageDistence-avgDistance)<5.0) {
						NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0,0);
						NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
						StageFlag = 0;
						StageIndex++;
					}
					else if (fabs(stageDistence-avgDistance)<70.0) {
						NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, (stageDistence-avgDistance)/70.0*stageSpeed, (stageDistence-avgDistance)/70.0*stageSpeed);
					}
					else {
						NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, stageSpeed, stageSpeed);
					}
					char temp[21];
						sprintf(temp, "len:%.2f",fabs(stageDistence-avgDistance));
						//Display_ShowString(2, 0, temp);
				}
			}
			default:
				break;
			}
			lastStageTime = nowTime;
			if (shouldStopRun) {
				break;
			}
		}

	}
}

#if MOTOR_STEP_TEST_ENABLE
static bool MotorStepTest_IsReached(float measured_mmps, float target_mmps) {
	float absTarget = fabsf(target_mmps);
	float absMeasured = fabsf(measured_mmps);
	float tolerance = absTarget * (1.0f - MOTOR_STEP_TEST_REACHED_RATIO);

	if (tolerance < MOTOR_STEP_TEST_ABS_TOL_MMPS) {
		tolerance = MOTOR_STEP_TEST_ABS_TOL_MMPS;
	}

	return fabsf(absTarget - absMeasured) <= tolerance;
}

static void MotorClosedLoopStepTest(void) {
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

	__disable_irq();
	motorLeftCount = 0;
	motorRightCount = 0;
	__enable_irq();
	leftDistance = 0;
	rightDistance = 0;

	NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0.0f, 0.0f);
	NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
	delay_ms(300);

	Display_Clear();
	Display_ShowString(0, 0, "Motor Step Test");
	{
		char oledLine[22];
		snprintf(oledLine, sizeof(oledLine), "Target:%d mm/s",
				 (int)targetMmps);
		Display_ShowString(2, 0, oledLine);
	}
	Display_ShowString(4, 0, "Measuring...");

	__disable_irq();
	motorLeftCount = 0;
	motorRightCount = 0;
	__enable_irq();

	commandTimeUs = getNowUs();
	lastUpdateUs = commandTimeUs;
	lastLogMs = getNowMs();
	NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, targetMmps, targetMmps);

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

			motor.sample_period_s =
				(float)getTimeUs(nowUs, lastUpdateUs) / 1000000.0f;
			lastActualPeriodUs = getTimeUs(nowUs, lastUpdateUs);
			lastUpdateUs = nowUs;
			updateCount++;

			__disable_irq();
			leftCountSnapshot = motorLeftCount;
			rightCountSnapshot = motorRightCount;
			motorLeftCount = 0;
			motorRightCount = 0;
			__enable_irq();

			leftDistance += leftCountSnapshot;
			rightDistance += rightCountSnapshot;
			measureLeftCount += leftCountSnapshot;
			measureRightCount += rightCountSnapshot;
			measureWindowUs += lastActualPeriodUs;

			if (!firstMoveSeen &&
				(leftCountSnapshot != 0 || rightCountSnapshot != 0)) {
				firstMoveSeen = true;
				firstMoveTimeUs = nowUs;
			}

			NewMotorSpeedCtrl_UpdateByEncoderDelta(
				&motor, leftCountSnapshot, rightCountSnapshot);
			NewMotorSpeedCtrl_GetMeasuredWheelMmps(&motor, &leftMmps,
												   &rightMmps);
			NewMotorSpeedCtrl_GetOutputPwmTicks(&motor, &leftPwm, &rightPwm);
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
				printf("Step t=%lu ms L=%d R=%d Avg=%d WinAvg=%d PWM=%d,%d Cnt=%ld,%ld dt=%lu us\n",
					   (unsigned long)(getTimeUs(nowUs, commandTimeUs) / 1000U),
					   (int)leftMmps, (int)rightMmps, (int)avgMmps,
					   (int)lastWindowAvgMmps, leftPwm, rightPwm,
					   (long)leftCountSnapshot,
					   (long)rightCountSnapshot,
					   (unsigned long)lastActualPeriodUs);
			}
		}
	}

	NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0.0f, 0.0f);
	NewMotor_Stop(NEWMOTOR_STOP_BRAKE);

	if (reached) {
		uint32_t commandToTargetMs =
			getTimeUs(reachedTimeUs, commandTimeUs) / 1000U;
		uint32_t motorStartToTargetMs =
			firstMoveSeen ? (getTimeUs(reachedTimeUs, firstMoveTimeUs) /
							 1000U)
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
		snprintf(oledLine, sizeof(oledLine), "T:%d A:%d",
				 (int)targetMmps, (int)lastWindowAvgMmps);
		Display_ShowString(4, 0, oledLine);
		snprintf(oledLine, sizeof(oledLine), "PWM:%d,%d", lastLeftPwm,
				 lastRightPwm);
		Display_ShowString(6, 0, oledLine);
		snprintf(oledLine, sizeof(oledLine), "PWM:%s",
				 pwmSaturated ? "Hit 2000" : "No limit");
		Display_ShowString(7, 0, oledLine);

		printf("Step Timeout target=%d elapsed=%lu ms updates=%lu last_dt=%lu us\n",
			   (int)targetMmps, (unsigned long)elapsedMs,
			   (unsigned long)updateCount, (unsigned long)lastActualPeriodUs);
		printf("Step Final instant_left=%d instant_right=%d instant_avg=%d window_left=%d window_right=%d window_avg=%d pwm=%d,%d last_cnt=%ld,%ld total_cnt=%ld,%ld\n",
			   (int)lastLeftMmps, (int)lastRightMmps,
			   (int)lastAvgMmps, (int)lastWindowLeftMmps,
			   (int)lastWindowRightMmps, (int)lastWindowAvgMmps,
			   lastLeftPwm, lastRightPwm,
			   (long)lastLeftCount, (long)lastRightCount,
			   (long)leftDistance, (long)rightDistance);
		printf("Step PWM saturated=%s threshold=%d\n",
			   pwmSaturated ? "yes" : "no",
			   MOTOR_STEP_TEST_PWM_SAT_TICKS);
		printf("Step Resolution control_period=%lu us one_count=%d mm/s measure_window=%lu us window_one_count=%d mm/s dropped_tx=%lu\n",
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

// MSPM0 的 GPIOA/GPIOB 外部中断属于 GROUP1 向量，
// 这里做一次分发，避免中断落入默认处理函数导致“卡死”。
void GROUP1_IRQHandler(void) {
	bool m1_A, m1_B, m2_A, m2_B;
	int gpioA_iidx, gpioB_iidx;

	// 分别查询两个 PORT 的待处理中断
	gpioA_iidx = DL_GPIO_getPendingInterrupt(GPIOA);
	gpioB_iidx = DL_GPIO_getPendingInterrupt(GPIOB);
	if (gpioA_iidx == MotorMonitor_E1A_IIDX) {
		DL_GPIO_clearInterruptStatus(MotorMonitor_E1A_PORT,
									 MotorMonitor_E1A_PIN);
		m1_A = (DL_GPIO_readPins(MotorMonitor_E1A_PORT, MotorMonitor_E1A_PIN) !=
				0);
		m1_B = (DL_GPIO_readPins(MotorMonitor_E1B_PORT, MotorMonitor_E1B_PIN) !=
				0);
		if (m1_A == m1_B)
			motorLeftCount--;
		else
			motorLeftCount++;
	}
	if (gpioB_iidx == MotorMonitor_E2A_IIDX) {
		DL_GPIO_clearInterruptStatus(MotorMonitor_E2A_PORT,
									 MotorMonitor_E2A_PIN);
		m2_A = (DL_GPIO_readPins(MotorMonitor_E2A_PORT, MotorMonitor_E2A_PIN) !=
				0);
		m2_B = (DL_GPIO_readPins(MotorMonitor_E2B_PORT, MotorMonitor_E2B_PIN) !=
				0);
		if (m2_A == m2_B)
			motorRightCount++;
		else
			motorRightCount--;
	}
	if (gpioB_iidx == key_PIN_B23_IIDX) {
		DL_GPIO_clearInterruptStatus(key_PORT, key_PIN_B23_PIN);
		TextIndex++;
		if (TextIndex > 3) {
			TextIndex -= 4;
		}
	}
	if (gpioB_iidx == key_PIN_B26_IIDX) {
		DL_GPIO_clearInterruptStatus(key_PORT, key_PIN_B26_PIN);
	}
}

// 串口的中断服务函数
void UART_0_INST_IRQHandler(void) {
	USART_IRQHandler();
}

static void BluetoothSerial_OnRxByte(uint8_t receivedData) {
	// DMA 每收到 1 字节回调一次，保持原 UART0 接收缓冲语义。
	if (recv0_length < 128 - 1 && receivedData != '\0' &&
		receivedData != '\n') {
		recv0_buff[recv0_length++] = receivedData;
	} else {
		recv0_buff[recv0_length] = '\0';
		recv0_length = 0;
	}
	recv0_flag = 1;
}
// maixcam的串口中断服务
void UART_MAIXCAM_INST_IRQHandler(void) {
	switch (DL_UART_getPendingInterrupt(UART_MAIXCAM_INST)) {
	case DL_UART_IIDX_RX: {
		uint8_t data = DL_UART_Main_receiveData(UART_MAIXCAM_INST);
		if (maixcam_length < 31 && data != '\0' && data != '\n') {
			maixcam_buff[maixcam_length++] = data;
		} else {
			maixcam_buff[maixcam_length] = '\0';
			maixcam_flag = 1;
			maixcam_length = 0;
		}
		break;
	}
	default:
		break;
	}
}
// 蜂鸣器鸣响三声
void buzzer_beep(void) {
	for (int i = 0; i < 3; i++) {
		DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_16); // 打开蜂鸣器
		DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_22);
		delay_ms(100);
		DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_16); // 关闭蜂鸣器
		DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_22);
		delay_ms(100);
	}
}
