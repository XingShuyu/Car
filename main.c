#include "ti_msp_dl_config.h"
#include "BasicMicroLib/delay.h"
#include "BasicMicroLib/usart.h"
#include "GrayScale/Grayscale_Scan.h"
#include "Motor/motor.h"
#include "MCU6050/mpu6050.h"
#include "BasicMicroLib/getTime.h"
#include <stdio.h>
#include <stdbool.h>

//循迹pid
PID garyscalePid = {1.0f,1.0f,1.0f,100.0,0};

//电机pid
PID motorPid = {1.0f,1.0f,1.0f,100.0,0};

//-------------------
//各种时间声明
//获取电机速度时间戳
uint32_t lastMotorSpeedTime=0;
//数据输出时间戳
uint32_t lastUartTime=0;
//循迹时间戳
uint32_t lastGrayscaleTime=0;

uint16_t grayscale[8];

volatile int32_t motorRightSpeed = 0;
volatile int32_t motorLeftSpeed = 0;
volatile int32_t motorLeftCount = 0;
volatile int32_t motorRightCount = 0;

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
	for (volatile uint32_t i = 0; i < 3200000; i++);

	SYSCFG_DL_init(); // 由SysConfig自动生成的初始化函数
	// 开启 GPIOA 和 GPIOB 的全局中断 (因为编码器引脚跨越了这两个端口)
	NVIC_EnableIRQ(MotorMonitor_GPIOA_INT_IRQN);
	NVIC_EnableIRQ(MotorMonitor_GPIOB_INT_IRQN);
	USART_Init(); // 使能UART中断（接收依赖此步骤）
	/*
	 * 修改2（最关键）：必须同时启动两个定时器！
	 * 根据你的 SysConfig，左电机绑定了 TIMG8，右电机绑定了 TIMG6。
	 * 如果宏名字报错，请去 ti_msp_dl_config.h 里搜索 TIMG 找到准确的名字
	 * (也有可能被重命名为 PWM_MotorLeft_INST 等，取决于你SysConfig的命名)
	 */
	setvbuf(stdout, NULL, _IONBF, 0);
	TimeBase_Init();
	DL_TimerG_startCounter(MotorLeft_INST);
	DL_TimerG_startCounter(MotorRight_INST);

	// 打印启动信息
	printf("MSPM0G3507 D157B Motor Test Start!\r\n");

	// 获取启动时间tick
	startTime = getNowMs();
	Motor_SetAccuSpeed(15, 15);

	// 时间轴开始
	while (1) {
		// 更新当前时间
		nowTime = getNowMs();
		// 每100ms获取电机运行圈数
		if (getTimeMs(nowTime, lastMotorSpeedTime) > 100) {
			int32_t leftCountSnapshot;
			int32_t rightCountSnapshot;

			lastMotorSpeedTime = nowTime;

			// 原子化读取并清零编码器计数，避免与中断并发导致丢脉冲
			__disable_irq();
			leftCountSnapshot = motorLeftCount;
			rightCountSnapshot = motorRightCount;
			motorLeftCount = 0;
			motorRightCount = 0;
			__enable_irq();

			motorRightSpeed = ((rightCountSnapshot * 10) / 4) / 28 / 500;
			motorLeftSpeed = ((leftCountSnapshot * 10) / 4) / 28 / 500;
			Motor_PidSpeed(&motorPid, motorLeftSpeed, motorRightSpeed);
		}

		if (getTimeMs(nowTime, lastUartTime) > 1000) {
			lastUartTime = nowTime;
			printf("电机1: %ld, 电机2: %ld\r\n", (long)motorRightSpeed,
				   (long)motorLeftSpeed);
		}

		//基础循迹
		if(getTimeMs(nowTime, lastGrayscaleTime) > 100){
			lastGrayscaleTime = nowTime
			Motor_FixError(Grayscale_Line(grayscale, &garyscalePid));

		}

		// float out = Grayscale_Line((uint16_t *)grayscale, &garyscalePid);
		// printf("pid = %.2f, 5: %u,4: %u,3: %u,2: %u\r\n",
		// out,grayscale[5],grayscale[4],grayscale[3],grayscale[2]); bool out =
		// Grayscale_Cross((uint16_t *)grayscale, (uint16_t)0.5, 1); printf("out
		// = %d, 7: %u,6: %u,1: %u,0: %u\r\n",
		// out,grayscale[7],grayscale[6],grayscale[1],grayscale[0]);
		// delay_ms(2000);

		/* 动作1：全速前进 */
		// Motor_SetSpeed(1000, 1000);
		// delay_ms(2000);
		// Motor_Brake();
		// delay_ms(2000);
		// Motor_SetSpeed(-1000,-1000);
		// delay_ms(2000);
		// MPU6050_ReadAll(&now);
		// printf("MCU: ax:%.2f, ay:%.2f, az:%.2f, gx:%.2f, gy:%.2f, gz:%.2f,
		// temp:
		// %.2f \r\n",now.ax,now.ay,now.az,now.gx,now.gy,now.gz,now.temp);
		// delay_ms(2000);
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
		DL_GPIO_clearInterruptStatus(MotorMonitor_E1A_PORT,MotorMonitor_E1A_PIN);
		m1_A = (DL_GPIO_readPins(MotorMonitor_E1A_PORT, MotorMonitor_E1A_PIN) !=
				0);
		m1_B = (DL_GPIO_readPins(MotorMonitor_E1B_PORT, MotorMonitor_E1B_PIN) !=
				0);
		if (m1_A == m1_B)
			motorLeftCount++;
		else
			motorLeftCount--;
	}
	if (gpioA_iidx == MotorMonitor_E1B_IIDX) {
		DL_GPIO_clearInterruptStatus(MotorMonitor_E1B_PORT,
									 MotorMonitor_E1B_PIN);
		m1_A = (DL_GPIO_readPins(MotorMonitor_E1A_PORT, MotorMonitor_E1A_PIN) !=
				0);
		m1_B = (DL_GPIO_readPins(MotorMonitor_E1B_PORT, MotorMonitor_E1B_PIN) !=
				0);
		if (m1_A != m1_B)
			motorLeftCount++;
		else
			motorLeftCount--;
	}
	if (gpioA_iidx == MotorMonitor_E2B_IIDX) {
		DL_GPIO_clearInterruptStatus(MotorMonitor_E2B_PORT,
									 MotorMonitor_E2B_PIN);
		m2_A = (DL_GPIO_readPins(MotorMonitor_E2A_PORT, MotorMonitor_E2A_PIN) !=
				0);
		m2_B = (DL_GPIO_readPins(MotorMonitor_E2B_PORT, MotorMonitor_E2B_PIN) !=
				0);
		if (m2_A != m2_B)
			motorRightCount--;
		else
			motorRightCount++;
	}
	if (gpioB_iidx == MotorMonitor_E2A_IIDX) {
		DL_GPIO_clearInterruptStatus(MotorMonitor_E2A_PORT,
									 MotorMonitor_E2A_PIN);
		m2_A = (DL_GPIO_readPins(MotorMonitor_E2A_PORT, MotorMonitor_E2A_PIN) !=
				0);
		m2_B = (DL_GPIO_readPins(MotorMonitor_E2B_PORT, MotorMonitor_E2B_PIN) !=
				0);
		if (m2_A == m2_B)
			motorRightCount--;
		else
			motorRightCount++;
	}
}