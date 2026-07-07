#include "BasicMicroLib/delay.h"
#include "BasicMicroLib/getTime.h"
#include "BasicMicroLib/usart.h"
#include "Balance/balance_control.h"
#include "Emm/Emm.h"
#include "GrayScale/Grayscale_Scan.h"
#include "IMU/imu.h"
#include "Motor/newmotor_speed_ctrl.h"
#include "OLED/display.h"
#include "Stage.h"
#include "ti_msp_dl_config.h"
#include "wdd35d4/wd35d4.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RAD_TO_DEG 57.29578f   // 将弧度制转换为角度制
#define DEG_TO_RAD 0.01745329f // 角度制转化为弧度制
#define G_TO_MS2 9.8f		   // 加速度取9.8
#define DT_SAMPLE 0.01f		   // 采样周期10ms
#define PDNum 10
#define MOTOR_STEP_TEST_ENABLE 0
static float yaw_angle = 0.0f; // 偏航角（度），绕 Z 轴

// 循迹pid
PID grayscalePid = {0.1f, 0.0f, 0.0f, 100000.0, 0, 10};

// 基础速度
int BaseSpeed = 50;
int RoundSpeed = 50;
float distance;
// l1 l2距离
float distence[2];
// 黑线长度
int blackLen[3];
//-------------------
// 各种时间声明
// 获取电机速度时间戳
uint32_t lastMotorSpeedTime = 0;
// 位置环时间
uint32_t lastPositionSpeedTime = 0;
// 数据输出时间戳
uint32_t lastUartTime = 0;
// 循迹时间戳
uint32_t lastGrayscaleTime = 0;
// 超声波时间戳
uint32_t lastUltrasonicTime = 0;
// IMU时间戳
uint32_t lastIMUTime = 0;
// 阶段时间戳
uint32_t lastStageTime = 0;
// OLED时间戳
uint32_t lastOLEDTime = 0;
// 状态机下标
int TextIndex = 0;
// 终点下标
int Goal = 0;
// 阶段索引
int StageIndex = 0;
// 阶段标志位
int StageFlag = 0;
// 蓝牙时间戳
uint32_t lastBluetoothTime = 0;
// maixcam串口相关
volatile uint8_t maixcam_buff[32] = {0};
volatile uint16_t maixcam_length = 0;
volatile uint8_t maixcam_flag = 0;
// 串口相关
volatile uint8_t recv0_buff[128] = {0};
volatile uint16_t recv0_length = 0;
volatile uint8_t recv0_flag = 0;
// key
uint8_t key_last = 0;

// IMU相关
IMU_Data_t IMUData;

// WDD35D4角位移传感器实时数据
WDD35D4_Data_t WDD35D4Data;

// 灰度循迹地址
bool grayscale[8];

// volatile float motorRightSpeed = 0; //	速度(m/s)
// volatile float motorLeftSpeed = 0;	//	速度(m/s)
NewMotor_SpeedCtrl motor;
volatile int32_t motorLeftCount = 0;
volatile int32_t motorRightCount = 0;
int leftDistance, rightDistance;


void process_imu_for_horizontal_motion(float dt);
void Display_WheelSpeeds();
void buzzer_beep(void);
static void buzzer_set(bool enabled);
static void buzzer_notify_start(uint8_t beep_count);
static void buzzer_notify_update(uint32_t now_ms);
static bool buzzer_notify_is_active(void);
#if MOTOR_STEP_TEST_ENABLE
static void MotorClosedLoopStepTest(void);
static bool MotorStepTest_IsReached(float measured_mmps, float target_mmps);
#endif

static bool buzzer_notify_active = false;
static bool buzzer_notify_on = false;
static uint8_t buzzer_notify_edges_remaining = 0;
static uint32_t buzzer_notify_next_ms = 0;

int main(void) {
	//--------------------------------------
	//                初始化
	//--------------------------------------
	SYSCFG_DL_init(); // 由SysConfig自动生成的初始化函数
	//---------------中断使能----------------

	// 开启 GPIOA 和 GPIOB 的全局中断 (因为编码器引脚跨越了这两个端口)
	NVIC_EnableIRQ(MotorMonitor_GPIOA_INT_IRQN);
	NVIC_EnableIRQ(GPIOB_INT_IRQn);
	USART_Init(); // 使能UART中断（接收依赖此步骤）
	setvbuf(stdout, NULL, _IONBF, 0);

	Display_Init(); // 初始化显示屏

	NVIC_EnableIRQ(UART_MAIXCAM_INST_INT_IRQN); // 初始化maixcam

	TimeBase_Init(); // 初始化计时器

	// WDD35D4角位移传感器初始化。上电时保持摆杆竖直，优先自动采样当前值作为零点；
	// 若采样失败，则回退到头文件中的默认零点。
	WDD35D4_Init();
	WDD35D4_SetZeroRaw(990);

	// 使能云台
	Emm_Init(1);
	delay_ms(10);
	Emm_Init(2);
	delay_ms(10);

	// 电机初始化
	DL_TimerG_startCounter(MotorLeft_INST);
	DL_TimerG_startCounter(MotorRight_INST);
	NewMotorSpeedCtrl_Init(&motor, 0.001f);
	NewMotorSpeedCtrl_SetPid(&motor, 13.0, 800.0, 0.0);
	NewMotorSpeedCtrl_SetOutputLimit(&motor, -2000, 2000);
	NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0.0f, 0.0f);
	BalanceControl_Init();

	printf("OK");

#if MOTOR_STEP_TEST_ENABLE
	MotorClosedLoopStepTest();
#endif

	// IMU初始化
	{
		int temp = 0;
		temp = IMU_Init();
		if (temp) {
			char imuLine[20] = "IMU:";
			bool needSlash = false;
			if ((temp & IMU_DEVICE_MASK_HWT101) != 0) {
				strcat(imuLine, "HWT");
				needSlash = true;
			}
			if ((temp & IMU_DEVICE_MASK_JY901S) != 0) {
				if (needSlash) {
					strcat(imuLine, "/");
				}
				strcat(imuLine, "JY");
				needSlash = true;
			}
			if ((temp & IMU_DEVICE_MASK_MPU6050) != 0) {
				if (needSlash) {
					strcat(imuLine, "/");
				}
				strcat(imuLine, "MPU");
			}
			Display_ShowString(0, 0, imuLine);
		} else {
			Display_ShowString(0, 0, "IMU Faid");
		}
		delay_ms(1000);
	}

	// 获取启动时间tick
	Display_ShowString(0, 0, "Car Ready"); // 可选：开机显示欢迎信息
	delay_ms(500);
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
	int16_t *command = commandList[TextIndex];
	if (TextIndex == 3) {
		TextIndex = 0;
		startTime = getNowMs();
		while (getTimeMs(getNowMs(), startTime) < 1000) {
			char str[8];
			sprintf(str, "goal: %d", TextIndex);
			Display_ShowString(0, 0, str);
		}
		Goal = TextIndex;
	}
	startTime = getNowMs();
	lastIMUTime = getNowMs();
	lastStageTime = getNowMs()+5000;
	lastMotorSpeedTime = getNowUs();
	lastPositionSpeedTime = lastMotorSpeedTime;
	bool balanceReadyNotified = false;


	while (1) {
		// 更新当前时间
		nowTime = getNowMs();
		buzzer_notify_update(nowTime);
		if (BalanceControl_GetMode() != BALANCE_CONTROL_MODE_RUNNING) {
			balanceReadyNotified = false;
		} else if (!buzzer_notify_is_active() && !balanceReadyNotified) {
			balanceReadyNotified = true;
			lastStageTime = nowTime;
		}
		USART_PollTx();
		uint32_t nowUs = getNowUs();
		// 5ms平衡内环：主循环只负责取时间、读编码器和IMU，控制计算在Balance模块内完成
		if (getTimeUs(nowUs, lastMotorSpeedTime) >
			BALANCE_CONTROL_FAST_PERIOD_US) {

			// 获取速度
			int32_t leftCountSnapshot;
			int32_t rightCountSnapshot;
			float balanceDt =
				(float)getTimeUs(nowUs, lastMotorSpeedTime) / 1000000.0f;
			lastMotorSpeedTime = nowUs;

			// 原子化读取并清零编码器计数，避免与中断并发导致丢脉冲
			__disable_irq();
			leftCountSnapshot = motorLeftCount;
			rightCountSnapshot = motorRightCount;
			motorLeftCount = 0;
			motorRightCount = 0;
			__enable_irq();
			//-------左右轮同步转，不要删--------
			// leftCountSnapshot = rightCountSnapshot;
			//---------------------------------
			leftDistance += leftCountSnapshot;
			rightDistance += rightCountSnapshot;

			IMU_ReadAll(&IMUData);
			if (!BalanceControl_UpdateFast(&motor,
										   &IMUData,
										   leftCountSnapshot,
										   rightCountSnapshot,
										   balanceDt,
										   NULL)) {
				leftDistance = 0;
				rightDistance = 0;
			}
		}
		// 20ms平衡状态机/速度外环：启动时标定中心角，运行后只更新临时目标倾角
		if (getTimeUs(nowUs, lastPositionSpeedTime) >
			BALANCE_CONTROL_OUTER_PERIOD_US) {
			float dt =
				(float)getTimeUs(nowUs, lastPositionSpeedTime) / 1000000.0f;
			BalanceControl_Mode balanceModeBefore = BalanceControl_GetMode();

			lastPositionSpeedTime = nowUs;
			BalanceControl_UpdateTarget(&motor, &IMUData, dt);
			if (balanceModeBefore != BALANCE_CONTROL_MODE_RUNNING &&
				BalanceControl_GetMode() == BALANCE_CONTROL_MODE_RUNNING) {
				balanceReadyNotified = false;
				lastStageTime = nowTime;
				buzzer_notify_start(3);
			}
		}


		if (BalanceControl_IsRunning() && balanceReadyNotified &&
			getTimeMs(nowTime, lastStageTime) > 5) {
			int16_t stage = command[StageIndex];
			bool shouldStopRun = false;
			// NewMotorSpeedCtrl_SetTargetWheelMmps(&motor,BaseSpeed,BaseSpeed);
			//leftDistance = 0;

			switch (stage) {
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
			case Stageultrasonic: {
				// Stageultrasonic
				// if (StageFlag == 0) {
				// 	distance = 0.0;
				// 	distance = Ultrasonic_GetDistance();
				// 	if (distance != 0.0f) {
				// 		StageFlag++;
				// 	}
				// }
				// if (StageFlag == 1 && distance < 40.0f) {
				// 	NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
				// 	NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
				// 	StageFlag = 0;
				// 	StageIndex++;
				// }
				// if (StageFlag == 1 && distance >= 40.0f) {
				// 	StageFlag = 0;
				// 	StageIndex = sizeof(command) / sizeof(int16_t) - 1;
				// }
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

				NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);

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
				// break;
			}
			case StageTurn145: {
				if (StageFlag == 0) {
					IMUData.yaw = 0.0;
					IMU_ZeroYaw();
					StageFlag++;
				}
				NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, -(RoundSpeed),
													 (RoundSpeed));
				(void)IMU_ReadAll(&IMUData);
				if (IMUData.yaw > 132.0 && IMUData.yaw < 137.0) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
					NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
					StageFlag = 0;
					lastStageTime = nowTime;
					StageIndex++;
				} else {
					float error = (-IMUData.yaw + 135) / 135 + 0.3;
					NewMotorSpeedCtrl_SetTargetWheelMmps(
						&motor, -(error * RoundSpeed), (error * RoundSpeed));
				}
				break;
			}
			case StageSkip: {
				// StageSkip
				int offset = (3 - Goal) * 3;
				StageIndex += offset + 1;
				break;
			}
			default:
				break;
			}
			lastStageTime = nowTime;
			if (shouldStopRun) {
				break;
			}
		}

		// if (getTimeMs(nowTime, lastGrayscaleTime) > 50 &&
		// Grayscale_Cross(grayscale, 1)) { 	lastGrayscaleTime = nowTime;

		// }

		// // 基础循迹
		// if (getTimeMs(nowTime, lastGrayscaleTime) > 10) {
		// 	lastGrayscaleTime = nowTime;
		// 	Motor_FixError(Grayscale_Line(grayscale));
		// }
		// // 判断十字路口
		// if (Grayscale_Cross(grayscale, 0) == true) {
		// 	// 超声波测距
		// 	if (getTimeMs(nowTime, lastUltrasonicTime) > 1000) {
		// 		lastUltrasonicTime = nowTime;
		// 		distance = Ultrasonic_GetDistance();
		// 	}
		// }
		// uint32_t now = getNowMs();
		// float dt = (float)(now - last_time) / 1000.0f;
		// if (dt > 0.1f)
		// dt = 0.01f; // 限制最大 dt，防止突变
		// last_time = now;

		// // 处理 IMU 数据，更新偏航角、速度、位移
		// process_imu_for_horizontal_motion(dt);

		// // 延时到下一个周期（非精确，仅示例）
		// delay_ms((int)(DT_SAMPLE * 100));

		// if (getTimeMs(nowTime, lastGrayscaleTime) > 1000) {
		// 	lastGrayscaleTime = nowTime;
		// 	// 输出结果（可通过串口查看）
		// 	printf(
		// 		"Yaw: %.1f deg",yaw_angle);
		// }
		// if (maixcam_flag) {
		// 	maixcam_flag = 0;
		// 	// printf("RAW: ");
		// 	// for (int i = 0; i < maixcam_length && i < 20; i++) {
		// 	// 	printf("%02X ", maixcam_buff[i]);
		// 	// }
		// 	// printf("\n");
		// 	int off_x, off_y;
		// 	if (sscanf((char *)maixcam_buff, "%d,%d", &off_x, &off_y) ==
		// 		2) {
		// 		printf("[MaixCAM] X=%d Y=%d", off_x, off_y);
		// 	}
		// }
		// if (getTimeMs(nowTime, lastOLEDTime) > 1000) {
		// 	// // 显示左右轮速度
		// 	// Display_WheelSpeeds();
		// 	// Display_Clear();
		// }
	}
}

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
	uint8_t receivedData = 0;
	DL_UART_IIDX pending = DL_UART_getPendingInterrupt(UART_0_INST);

	// 如果产生了串口中断
	// If a serial port interrupt occurs
	switch (pending) {
	case DL_UART_IIDX_RX: // 如果是接收中断	If it is a receive interrupt

		while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
			// 接收发送过来的数据保存	Receive and save the data sent
			receivedData = DL_UART_Main_receiveData(UART_0_INST);

			// 检查缓冲区是否已满	Check if the buffer is full
			if (recv0_length < 128 - 1 && receivedData != '\0' &&
				receivedData != '\n') {
				recv0_buff[recv0_length++] = receivedData;
			} else {
				recv0_length = 0;
			}

			// 标记接收标志	Mark receiving flag
			recv0_flag = 1;
		}

		break;

	case DL_UART_IIDX_DMA_DONE_TX:
	case DL_UART_IIDX_EOT_DONE:
		USART_HandleTxInterrupt(pending);
		break;

	default: // 其他的串口中断	Other serial port interrupts
		break;
	}
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
// 计算姿态角和位移的函数(dt单位秒)
void process_imu_for_horizontal_motion(float dt) {
	IMU_Data_t data;
	if (!IMU_ReadAll(&data)) {
		printf("IMU read error\n");
		return;
	}
	if (data.gz < 0.01 && data.gz > -0.01) {

	} else {
		yaw_angle += data.gz * dt;
	}
}

static void buzzer_set(bool enabled) {
	if (enabled) {
		DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_16);
		DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_22);
	} else {
		DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_16);
		DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_22);
	}
}

static void buzzer_notify_start(uint8_t beep_count) {
	if (beep_count == 0U) {
		return;
	}

	buzzer_notify_active = true;
	buzzer_notify_on = false;
	buzzer_notify_edges_remaining = (uint8_t)(beep_count * 2U);
	buzzer_notify_next_ms = getNowMs();
	buzzer_set(false);
}

static void buzzer_notify_update(uint32_t now_ms) {
	if (!buzzer_notify_active) {
		return;
	}
	if ((int32_t)(now_ms - buzzer_notify_next_ms) < 0) {
		return;
	}
	if (buzzer_notify_edges_remaining == 0U) {
		buzzer_set(false);
		buzzer_notify_on = false;
		buzzer_notify_active = false;
		return;
	}

	buzzer_notify_on = !buzzer_notify_on;
	buzzer_set(buzzer_notify_on);
	buzzer_notify_edges_remaining--;
	buzzer_notify_next_ms = now_ms + 100U;

	if (buzzer_notify_edges_remaining == 0U && !buzzer_notify_on) {
		buzzer_notify_active = false;
	}
}

static bool buzzer_notify_is_active(void) {
	return buzzer_notify_active;
}

// 蜂鸣器阻塞鸣响三声，仅用于停车后的提示；平衡运行中使用 buzzer_notify_*。
void buzzer_beep(void) {
	for (int i = 0; i < 3; i++) {
		buzzer_set(true);
		delay_ms(100);
		buzzer_set(false);
		delay_ms(100);
	}
}
// // 显示左右轮速度
// void Display_WheelSpeeds() {
// 	char left_str[16], right_str[16];

// 	// 格式化左轮速度字符串
// 	sprintf(left_str, "L:%.2f m/s", motorLeftSpeed);
// 	// 格式化右轮速度字符串
// 	sprintf(right_str, "R:%.2f m/s", motorRightSpeed);

// 	// 在OLED上显示（左轮在上，右轮在下）
// 	Display_ShowString(0, 0, left_str);
// 	Display_ShowString(2, 0, right_str);

// 	// 可选：显示速度差
// 	float speed_diff = fabs(motorLeftSpeed - motorRightSpeed);
// 	sprintf(left_str, "Diff:%.2f m/s", speed_diff);
// 	Display_ShowString(4, 0, left_str);
// }
