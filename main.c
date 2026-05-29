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
#include "ultrasonic/ultrasonic.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RAD_TO_DEG 57.29578f   // 将弧度制转换为角度制
#define DEG_TO_RAD 0.01745329f // 角度制转化为弧度制
#define G_TO_MS2 9.8f		   // 加速度取9.8
#define DT_SAMPLE 0.01f		   // 采样周期10ms
static float yaw_angle = 0.0f; // 偏航角（度），绕 Z 轴

// 循迹pid
PID grayscalePid = {0.1f, 0.0f, 0.0f, 100000.0, 0, 10};

// 基础速度
int BaseSpeed = 450;
int RoundSpeed = 200;
float distance;
// l1 l2距离
float distence[2];
// 黑线长度
int blackLen[3];
//-------------------
// 各种时间声明
// 获取电机速度时间戳
uint32_t lastMotorSpeedTime = 0;
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

// 灰度循迹地址
bool grayscale[8];

// volatile float motorRightSpeed = 0; //	速度(m/s)
// volatile float motorLeftSpeed = 0;	//	速度(m/s)
NewMotor_SpeedCtrl motor;
volatile int32_t motorLeftCount = 0;
volatile int32_t motorRightCount = 0;
int leftDistance, rightDistance;

float Vx, Vy, Vz, Xx, Xy, Xz;

void process_imu_for_horizontal_motion(float dt);
void Display_WheelSpeeds();
void buzzer_beep(void);

int main(void) {
	//--------------------------------------
	//                初始化
	//--------------------------------------
	SYSCFG_DL_init(); // 由SysConfig自动生成的初始化函数
	//---------------中断使能----------------

	// 开启 GPIOA 和 GPIOB 的全局中断 (因为编码器引脚跨越了这两个端口)
	NVIC_EnableIRQ(MotorMonitor_GPIOA_INT_IRQN);
	NVIC_EnableIRQ(GPIOB_INT_IRQn);
	//--------------模块初始化---------------
	Ultrasonic_Init(); // 初始化超声波函数

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
	NewMotorSpeedCtrl_Init(&motor, 0.01f);
	NewMotorSpeedCtrl_SetPid(&motor, 2.0, 1.1, 1.0);
	NewMotorSpeedCtrl_SetOutputLimit(&motor, -2000, 2000);
	NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, BaseSpeed, BaseSpeed);

	// IMU初始化
	{
		int temp = 0;
		temp = IMU_Init();
		if (temp) {
			if (temp == 1) {
				Display_ShowString(0, 0, "JY901S Ready");
			} else if (temp == 2) {
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
	while (getTimeMs(getNowMs(), startTime) < 3000) {
		char str[8];
		sprintf(str, "set: %d", TextIndex);
		Display_ShowString(0, 0, str);
	}
	int16_t *command = commandList[TextIndex];
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
	lastIMUTime = getNowMs();
	NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 500, 500);

	while (1) {
		// 更新当前时间
		nowTime = getNowMs();
		// 每30ms获取电机运行圈数
		if (getTimeMs(nowTime, lastMotorSpeedTime) > 10) {
			int32_t leftCountSnapshot;
			int32_t rightCountSnapshot;
			motor.sample_period_s =
				(float)getTimeMs(nowTime, lastMotorSpeedTime) / 1000;
			lastMotorSpeedTime = nowTime;

			// 原子化读取并清零编码器计数，避免与中断并发导致丢脉冲
			__disable_irq();
			leftCountSnapshot = motorLeftCount;
			rightCountSnapshot = motorRightCount;
			motorLeftCount = 0;
			motorRightCount = 0;
			__enable_irq();
			leftDistance += leftCountSnapshot;
			rightDistance += rightCountSnapshot;

			NewMotorSpeedCtrl_UpdateByEncoderDelta(&motor, leftCountSnapshot,
												   rightCountSnapshot);
		}

		IMU_ReadAll(&IMUData);
		OLED_Clear();
		OLED_DrawCircle(64, 32, 30);
		int XOff = (IMUData.pitch/90.0) * 32;
		OLED_DrawCircle(64, 32 + XOff, 2);
		OLED_Update();

		if (getTimeMs(nowTime, lastStageTime) > 10) {
			int16_t stage = command[StageIndex];
			bool shouldStopRun = false;

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
				grayscalePid.t = getTimeMs(nowTime, lastStageTime);
				Grayscale_Line(&grayscalePid, grayscale);
				NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0.5 * -BaseSpeed,
													 0.5 * -BaseSpeed);
				if (grayscale[0] == 0 && grayscale[1] == 0 &&
					grayscale[2] == 0 && grayscale[3] == 0 &&
					grayscale[4] == 0 && grayscale[5] == 0 &&
					grayscale[6] == 0 && grayscale[7] == 0 && StageFlag == 0) {
					StageFlag++;
				}
				if (!(grayscale[0] == 0 && grayscale[1] == 0 &&
					  grayscale[2] == 0 && grayscale[3] == 0 &&
					  grayscale[4] == 0 && grayscale[5] == 0 &&
					  grayscale[6] == 0 && grayscale[7] == 0) &&
					StageFlag == 1) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
					NewMotor_Stop(NEWMOTOR_STOP_COAST);
					buzzer_beep();
					StageFlag = 0;
					shouldStopRun = true;
				}
				break;
			}
			case StageTurn145: {
				if (StageFlag == 0) {
					IMUData.yaw = 0.0;
					StageFlag++;
				}
				NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, -(RoundSpeed),
													 (RoundSpeed));
				if (IMUData.yaw > 132.0 && IMUData.yaw < 137.0) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
					NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
					StageFlag = 0;
					lastStageTime = nowTime;
					StageIndex++;
				} else {
					float t = getTimeMs(getNowMs(), lastStageTime);
					if (MPU6050_ReadAllCalibrated(&IMUData)) {
						IMUData.yaw += IMUData.gz * t / 1000.0f;
					}
					char str[16];
					sprintf(str, "ang:%.2f", IMUData.yaw);
					Display_ShowString(0, 0, str);
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
	if (gpioA_iidx == MotorMonitor_E1B_IIDX) {
		DL_GPIO_clearInterruptStatus(MotorMonitor_E1B_PORT,
									 MotorMonitor_E1B_PIN);
		m1_A = (DL_GPIO_readPins(MotorMonitor_E1A_PORT, MotorMonitor_E1A_PIN) !=
				0);
		m1_B = (DL_GPIO_readPins(MotorMonitor_E1B_PORT, MotorMonitor_E1B_PIN) !=
				0);
		if (m1_A != m1_B)
			motorLeftCount--;
		else
			motorLeftCount++;
	}
	if (gpioA_iidx == MotorMonitor_E2B_IIDX) {
		DL_GPIO_clearInterruptStatus(MotorMonitor_E2B_PORT,
									 MotorMonitor_E2B_PIN);
		m2_A = (DL_GPIO_readPins(MotorMonitor_E2A_PORT, MotorMonitor_E2A_PIN) !=
				0);
		m2_B = (DL_GPIO_readPins(MotorMonitor_E2B_PORT, MotorMonitor_E2B_PIN) !=
				0);
		if (m2_A != m2_B)
			motorRightCount++;
		else
			motorRightCount--;
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
		TextIndex++;
		if (TextIndex > 3) {
			TextIndex -= 4;
		}
	}
}

// 串口的中断服务函数
void UART_0_INST_IRQHandler(void) {
	uint8_t receivedData = 0;

	// 如果产生了串口中断
	// If a serial port interrupt occurs
	switch (DL_UART_getPendingInterrupt(UART_0_INST)) {
	case DL_UART_IIDX_RX: // 如果是接收中断	If it is a receive interrupt

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
	JY901S_Data_t data;
	if (!MPU6050_ReadAllCalibrated(&data)) {
		printf("MPU6050 read error\n");
		return;
	}
	if (data.gz < 0.01 && data.gz > -0.01) {

	} else {
		yaw_angle += data.gz * dt;
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
