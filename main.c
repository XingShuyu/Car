#include "BasicMicroLib/delay.h"
#include "BasicMicroLib/getTime.h"
#include "BasicMicroLib/usart.h"
#include "GrayScale/Grayscale_Scan.h"
#include "MCU6050/mpu6050.h"
#include "Motor/motor.h"
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

// 电机pid 0.003
PID motorPid = {0.34f, 0.0005f, 0.00001f, 1000000.0, 0, 50};

// 基础速度
int BaseSpeed = 5000;
//障碍物距离
float distance;
//-------------------
// 各种时间声明
// 获取电机速度时间戳
uint32_t lastMotorSpeedTime = 0;
// 数据输出时间戳
uint32_t lastUartTime = 0;
// 循迹时间戳
uint32_t lastGrayscaleTime = 0;
//超声波时间戳
uint32_t lastUltrasonicTime=0;
// IMU时间戳
uint32_t lastIMUTime = 0;
// 阶段时间戳
uint32_t lastStageTime = 0;
// 阶段索引
int StageIndex = 0;
// 阶段标志位
int StageFlag = 0;
// 蓝牙时间戳
uint32_t lastBluetoothTime = 0;

volatile uint8_t recv0_buff[128] = {0};
volatile uint16_t recv0_length = 0;
MPU6050_Data_t data;
volatile uint8_t recv0_flag = 0;
bool grayscale[8];

volatile int32_t motorRightSpeed = 0;
volatile int32_t motorLeftSpeed = 0;
volatile int32_t motorLeftCount = 0;
volatile int32_t motorRightCount = 0;

void process_imu_for_horizontal_motion(float dt);
void buzzer_beep(void);
void CarRight(void) {
	Motor_Brake();
	delay_ms(1000);
	Motor_SetSpeed(1000, -1000);
	delay_ms(400);
	Motor_SetSpeed(600, 1000);
	delay_ms(4000);
	Motor_SetSpeed(1000, -1000);
	delay_ms(400);
	Motor_Brake();
}

int main(void) {
	//--------------------------------------
	//                 初始化
	//--------------------------------------
	for (volatile uint32_t i = 0; i < 3200000; i++)
		;

	SYSCFG_DL_init(); // 由SysConfig自动生成的初始化函数
	// 开启 GPIOA 和 GPIOB 的全局中断 (因为编码器引脚跨越了这两个端口)
	NVIC_EnableIRQ(MotorMonitor_GPIOA_INT_IRQN);
	NVIC_EnableIRQ(MotorMonitor_GPIOB_INT_IRQN);
	Ultrasonic_Init();//初始化超声波函数
	USART_Init(); // 使能UART中断（接收依赖此步骤）
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

	// 获取启动时间tick
	startTime = getNowMs();
	// Rush();
	Motor_SetAccuSpeed(60000, 60000);
	// RightRound();

	// 时间轴开始
	// buzzer_beep();
	// 初始化 MPU6050（默认 ±2g / ±250°/s）
	MPU6050_Init();
	while (1) {
		// 更新当前时间
		nowTime = getNowMs();
		// 每10ms获取电机运行圈数
		if (getTimeMs(nowTime, lastMotorSpeedTime) > 30) {
			int32_t leftCountSnapshot;
			int32_t rightCountSnapshot;
			motorPid.t = getTimeMs(nowTime, lastMotorSpeedTime);
			lastMotorSpeedTime = nowTime;

			// 原子化读取并清零编码器计数，避免与中断并发导致丢脉冲
			__disable_irq();
			leftCountSnapshot = motorLeftCount;
			rightCountSnapshot = motorRightCount;
			motorLeftCount = 0;
			motorRightCount = 0;
			__enable_irq();
			leftCountSnapshot = leftCountSnapshot / motorPid.t * 500;
			rightCountSnapshot = rightCountSnapshot / motorPid.t * 500;

			// motorRightSpeed = rightCountSnapshot/95;
			// motorLeftSpeed = leftCountSnapshot/95;

			Motor_PidSpeed(&motorPid, leftCountSnapshot, rightCountSnapshot);
		}
		if (getTimeMs(nowTime, lastIMUTime) > 50) {
			lastIMUTime = nowTime;


			
			MPU6050_ReadAll(&data);
			// printf("ax:%f",data.ax);
		}

		// if (getTimeMs(nowTime, lastStageTime) > 10) {
		// 	if (command[StageIndex] == 1) {
		// 		// RUSH
		// 		if (StageFlag == 0) {
		// 			Motor_SetAccuSpeed(5000, 5000);
		// 			StageFlag++;
		// 		}
		// 		if (StageFlag < 10) {
		// 			StageFlag++;
		// 		} else {
		// 			Motor_SetAccuSpeed(0, 0);
		// 			StageFlag = 0;
		// 			StageIndex++;
		// 		}
		// 	}
		// 	if (command[StageIndex] == 2) {
		// 		if (StageFlag == 0) {
		// 			Motor_SetAccuSpeed(BaseSpeed, BaseSpeed);
		// 			StageFlag++;
		// 		}
		// 		if (StageFlag == 1 && Grayscale_Cross(grayscale, 1)) {
		// 			Motor_SetAccuSpeed(0, 0);
		// 			Motor_Brake();
		// 			StageFlag = 0;
		// 			StageIndex++;
		// 		}
		// 	}
		// 	if (command[StageIndex] == 3) {
		// 		if (StageFlag == 0) {
		// 			Motor_SetAccuSpeed(5000, -5000);
		// 			StageFlag++;
		// 		}
		// 		if (StageFlag == 1 && _read_channel_stable(1)) {
		// 			Motor_SetAccuSpeed(0, 0);
		// 			Motor_Brake();
		// 			StageFlag = 0;
		// 			StageIndex++;
		// 		}
		// 	}
		// 	lastStageTime = nowTime;
		// }

		// if (getTimeMs(nowTime, lastGrayscaleTime) > 50 &&
		// Grayscale_Cross(grayscale, 1)) { 	lastGrayscaleTime = nowTime;

		// }

		// // 基础循迹
		//  if(getTimeMs(nowTime, lastGrayscaleTime) > 10){
		//  	lastGrayscaleTime = nowTime;
		//  	Motor_FixError(Grayscale_Line(grayscale, &garyscalePid));

		// }
		//超声波测距
		if(getTimeMs(nowTime, lastUltrasonicTime) > 1000){
			lastUltrasonicTime=nowTime;
            distance=Ultrasonic_GetDistance();
			if(distance>20.0 || distance == 0.0){
				printf("前方无障碍\r\n");
			}
			else{
				printf("前方有障碍且障碍距离为%.1f cm\r\n",distance);
			}
		}
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