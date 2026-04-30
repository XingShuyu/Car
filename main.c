#include "BasicMicroLib/delay.h"
#include "BasicMicroLib/getTime.h"
#include "BasicMicroLib/usart.h"
#include "Emm/Emm.h"
#include "GrayScale/Grayscale_Scan.h"
#include "MCU6050/mpu6050.h"
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
int BaseSpeed = 300;
int RoundSpeed = 200;
// 障碍物距离
float distance;
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
//状态机下标
uint8_t Stage_index=0;
//key
uint8_t key_last=0;

// MPU相关
MPU6050_RawData_t MPU6050Data;
float nowAngle = 0;

// 灰度循迹地址
bool grayscale[8];

volatile float motorRightSpeed = 0; //	速度(m/s)
volatile float motorLeftSpeed = 0;	//	速度(m/s)
volatile int32_t motorLeftCount = 0;
volatile int32_t motorRightCount = 0;
int leftDistance, rightDistance;

void process_imu_for_horizontal_motion(float dt);
void Display_WheelSpeeds();
void buzzer_beep(void);

int main(void) {
	//--------------------------------------
	//                 初始化
	//--------------------------------------

	SYSCFG_DL_init(); // 由SysConfig自动生成的初始化函数
	// 开启 GPIOA 和 GPIOB 的全局中断 (因为编码器引脚跨越了这两个端口)
	NVIC_EnableIRQ(MotorMonitor_GPIOA_INT_IRQN);
	NVIC_EnableIRQ(MotorMonitor_GPIOB_INT_IRQN);
	Ultrasonic_Init(); // 初始化超声波函数
	USART_Init();	   // 使能UART中断（接收依赖此步骤）

	// 初始化显示屏
	Display_Init();
	// 初始化maixcam
	NVIC_EnableIRQ(UART_MAIXCAM_INST_INT_IRQN);
	// 可选：开机显示欢迎信息
	Display_ShowString(0, 0, "Car Ready");
	delay_ms(2000);
	Display_Clear();

	/*
	 * 修改2（最关键）：必须同时启动两个定时器！
	 * 根据你的 SysConfig，左电机绑定了 TIMG8，右电机绑定了 TIMG6。
	 * 如果宏名字报错，请去 ti_msp_dl_config.h 里搜索 TIMG 找到准确的名字
	 * (也有可能被重命名为 PWM_MotorLeft_INST 等，取决于你SysConfig的命名)
	 */
	setvbuf(stdout, NULL, _IONBF, 0);
	TimeBase_Init();
	MPU6050_Init();
	DL_TimerG_startCounter(MotorLeft_INST);
	DL_TimerG_startCounter(MotorRight_INST);

	// 打印启动信息
	printf("MSPM0G3507 D157B Motor Test Start!\r\n");
	// 使能云台
	Emm_Init(1);
	delay_ms(10);
	Emm_Init(2);
	delay_ms(10);

	
	// 电机初始化
	NewMotor_SpeedCtrl motor;
	NewMotorSpeedCtrl_Init(&motor, 0.03f);
	NewMotorSpeedCtrl_SetPid(&motor, 2.0, 1.1, 1.0);
	NewMotorSpeedCtrl_SetOutputLimit(&motor, -2000, 2000);
	// NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, BaseSpeed, BaseSpeed);
	// 初始化 MPU6050（默认 ±2g / ±250°/s）
	MPU6050_Init();
	// 获取启动时间tick
	startTime = getNowMs();
	while(getTimeMs(startTime, getNowMs())<3000){
		uint8_t key_curr=DL_GPIO_readPins(key_PORT,key_PIN_B23_PIN);
        if(key_curr!=key_last){
			key_last=key_curr;
			Stage_index++;
		}
	}
	Stage_index/=2;
	Stage_index--;
	while (1) {
		// 更新当前时间
		nowTime = getNowMs();
		// 每30ms获取电机运行圈数
		if (getTimeMs(nowTime, lastMotorSpeedTime) > 30) {
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

			motorRightSpeed =
				(float)rightCountSnapshot / 0.03 / 4 / 500 / 28 * 204.2;
			motorLeftSpeed =
				(float)leftCountSnapshot / 0.03 / 4 / 500 / 28 * 204.2;
			leftDistance += leftCountSnapshot;
			rightDistance += rightCountSnapshot;

			NewMotorSpeedCtrl_UpdateByEncoderDelta(&motor, leftCountSnapshot,
												   rightCountSnapshot);
		}
		if (getTimeMs(nowTime, lastIMUTime) > 10) {

			int32_t t = getTimeMs(nowTime, lastIMUTime);
			lastIMUTime = nowTime;
			MPU6050_ReadGyroRaw(&MPU6050Data);
			nowAngle += MPU6050Data.z * t / 1000.0;
			LocControl locControl;
			// if (MPU6050Data.z > 0) {
			// 	locControl.accu = 0;
			// 	locControl.angle = nowAngle;
			// 	locControl.dirction = 0;
			// 	locControl.mode = 0;
			// 	locControl.speed = MPU6050Data.z;
			// 	Emm_Stop(2);
			// 	Emm_Loc_Control(2, &locControl);
			// } else if(MPU6050Data.z < 0){
			// 	locControl.accu = 0;
			// 	locControl.angle = -nowAngle;
			// 	locControl.dirction = 1;
			// 	locControl.mode = 0;
			// 	locControl.speed = -MPU6050Data.z;
			// 	Emm_Stop(2);
			// 	Emm_Loc_Control(2, &locControl);
			// }
			// else {
			// 	Emm_Stop(2);
			// }
			locControl.accu = 0;
			float angle = fmodf(nowAngle, 360.0f);
			if (angle < 0) {
				angle += 360.0f;
			}
			locControl.angle = angle;
			locControl.dirction = 0;
			locControl.mode = 1;
			locControl.speed = 1500;
			// Emm_Stop(2);
			Emm_Loc_Control(2, &locControl);
			char msg[16];
			sprintf(msg, "%d,%f", MPU6050Data.z, fmod(nowAngle, 360.0));
			Display_ShowString(0, 0, msg);
			Display_Clear();
		}

		if (getTimeMs(nowTime, lastStageTime) > 10) {
			if (command[StageIndex] == 1) {
				// 猛冲一下
				if (StageFlag == 0) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, BaseSpeed,
														 BaseSpeed);
					rightDistance = 0;
					leftDistance = 0;
					StageFlag++;
				}
				if (StageFlag == 1 &&
					(NewMotor_EncoderDeltaToDistanceMm(leftDistance) > 40 &&
					 NewMotor_EncoderDeltaToDistanceMm(rightDistance) > 40)) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
					NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
					StageFlag = 0;
					StageIndex++;
				}
			if (command[StageIndex] == 2) {
				// StageRight
				if (StageFlag == 0) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, BaseSpeed,
														 BaseSpeed);
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
			}
			if (command[StageIndex] == 3) {
				// StageRightRound
				if (StageFlag == 0) {
					StageFlag++;
				}
				NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, (RoundSpeed),
													 -(RoundSpeed));
				if (_read_channel_stable(3)) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
					NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
					StageFlag = 0;
					lastStageTime = nowTime;
					StageIndex++;
				}
			}
			if (command[StageIndex] == 4) {
				// StageLeft
				if (StageFlag == 0) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, BaseSpeed,
														 BaseSpeed);
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
			}
			if (command[StageIndex] == 5) {
				// StageLeftRound
				if (StageFlag == 0) {
					StageFlag++;
				}
				NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, (RoundSpeed),
													 -(RoundSpeed));
				if (_read_channel_stable(4)) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
					NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
					StageFlag = 0;
					lastStageTime = nowTime;
					StageIndex++;
				}
			}
			if (command[StageIndex] == 6) {
				// StageCross
				if (StageFlag == 0) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, BaseSpeed,
														 BaseSpeed);
					StageFlag++;
				}
				if (!Grayscale_Cross(grayscale, 0)) {
					grayscalePid.t = getTimeMs(nowTime, lastStageTime);
					float irr = Grayscale_Line(&grayscalePid, grayscale);
					NewMotorSpeedCtrl_SetTargetRobot(&motor, BaseSpeed, irr);

				} else {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
					NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
					StageFlag = 0;
					StageIndex++;
				}
			}
			if (command[StageIndex] == 7) {
				// Stageultrasonic
				if (StageFlag == 0) {
					distance = 0.0;
					distance = Ultrasonic_GetDistance();
					if (distance != 0.0f) {
						StageFlag++;
					}
				}
				if (StageFlag == 1 && distance < 40.0f) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
					NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
					StageFlag = 0;
					StageIndex++;
				}
				if (StageFlag == 1 && distance >= 40.0f) {
					StageFlag = 0;
					StageIndex = sizeof(command) / sizeof(int16_t) - 1;
				}
			}
			if (command[StageIndex] == 8) {
				// StageStartJudge
				if (Grayscale_Cross(grayscale, 1)) {
					// 右直角, AD起点
					StageFlag = 0;
					StageIndex++;

				} else if (Grayscale_Cross(grayscale, 2)) {
					// 左直角, AB起点
					StageFlag = 0;
					StageIndex++;
				} else {
				}
			}
			if (command[StageIndex] == 9) {
				// StageFinsih
				grayscalePid.t = getTimeMs(nowTime, lastStageTime);
				float irr = Grayscale_Line(&grayscalePid, grayscale);
				NewMotorSpeedCtrl_SetTargetRobot(&motor, BaseSpeed, irr);
				if (grayscale[0] == 0 && grayscale[1] == 0 &&
					grayscale[2] == 0 && grayscale[3] == 0 &&
					grayscale[4] == 0 && grayscale[5] == 0 &&
					grayscale[6] == 0 && grayscale[7] == 0) {
					NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
					NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
					break;
				}
			}
			if (command[StageIndex] == 10) {
				// StageBizz
				NewMotorSpeedCtrl_SetTargetWheelMmps(&motor, 0, 0);
				NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
				buzzer_beep();
				StageIndex++;
			}
			if (command[StageIndex] == 11) {
				// StageFake
				if(_read_channel_stable(3)||_read_channel_stable(4)||_read_channel_stable(2)||_read_channel_stable(5)){
					StageIndex-=2;
				}
				else {
					StageIndex++;
				}
			}
			lastStageTime = nowTime;
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
		if (maixcam_flag) {
			maixcam_flag = 0;
			// printf("RAW: ");
			// for (int i = 0; i < maixcam_length && i < 20; i++) {
			// 	printf("%02X ", maixcam_buff[i]);
			// }
			// printf("\n");
			int off_x, off_y;
			if (sscanf((char *)maixcam_buff, "%d,%d", &off_x, &off_y) == 2) {
				printf("[MaixCAM] X=%d Y=%d", off_x, off_y);
			}
		}
		if (getTimeMs(nowTime, lastOLEDTime) > 1000) {
			// // 显示左右轮速度
			// Display_WheelSpeeds();
			// Display_Clear();
		}
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
	MPU6050_Data_t data;
	if (!MPU6050_ReadAll(&data)) {
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
		DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_16); // 关闭蜂鸣器
		delay_ms(100);
		DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_16); // 打开蜂鸣器
		delay_ms(100);
	}
}
// 显示左右轮速度
void Display_WheelSpeeds() {
	char left_str[16], right_str[16];

	// 格式化左轮速度字符串
	sprintf(left_str, "L:%.2f m/s", motorLeftSpeed);
	// 格式化右轮速度字符串
	sprintf(right_str, "R:%.2f m/s", motorRightSpeed);

	// 在OLED上显示（左轮在上，右轮在下）
	Display_ShowString(0, 0, left_str);
	Display_ShowString(2, 0, right_str);

	// 可选：显示速度差
	float speed_diff = fabs(motorLeftSpeed - motorRightSpeed);
	sprintf(left_str, "Diff:%.2f m/s", speed_diff);
	Display_ShowString(4, 0, left_str);
}