#include "main.h"

volatile uint16_t grayscale[8];

volatile int32_t motor1_count = 0;
volatile int32_t motor1_speed = 0;
volatile int32_t motor2_count = 0;
volatile int32_t motor2_speed = 0;

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
	/*
	 * 关键修复：在系统初始化前加入延时。
	 * MSPM0在自动生成的 SYSCFG_DL_init()
	 * 中会复位GPIOA，这会导致SWD调试引脚瞬时重置。
	 * 如果上电后代码立即执行到这里，SWD连接会被切断，导致下载器无法连接并报错“锁死”。
	 * 添加1-2秒的延时，能给调试器留出充足的连接和暂停CPU的时间。
	 */
	for (volatile uint32_t i = 0; i < 3200000; i++)
		;

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

	// 时间轴开始
	while (1) {
		// 更新当前时间
		nowTime = getNowMs();
		// 每100ms获取电机运行圈数
		if (getTimeMs(nowTime, lastMotorSpeedTime) > 100) {
			lastMotorSpeedTime = nowTime;
			motor1_speed = motor1_count * 10;
			motor2_speed = motor2_count * 10;
			motor1_count = 0;
			motor2_count = 0;
		}

		if (getTimeMs(nowTime, lastUartTime) > 1000) {
			lastUartTime = nowTime;
			printf("电机1: %ld, 电机2: %ld\r\n", (long)motor1_speed,
				   (long)motor2_speed);
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

// void GPIOA_IRQHandler(void) {
// 	// 获取当前触发中断的具体引脚 (掩码)

// 	uint32_t pending_pins = DL_GPIO_getPendingInterrupt(MotorMonitor_E1A_PORT);

// 	// printf("pending_pins: %u, all: %d",pending_pins,(MotorMonitor_E1A_PIN |
// 	// MotorMonitor_E1B_PIN));
// 	// --- 处理电机 1 (E1A 和 E1B 都在 PORTA) ---
// 	if (pending_pins & (MotorMonitor_E1A_PIN | MotorMonitor_E1B_PIN)) {
// 		bool m1_A = (DL_GPIO_readPins(MotorMonitor_E1A_PORT,
// 									  MotorMonitor_E1A_PIN) != 0);
// 		bool m1_B = (DL_GPIO_readPins(MotorMonitor_E1B_PORT,
// 									  MotorMonitor_E1B_PIN) != 0);
// 		if (pending_pins & MotorMonitor_E1A_PIN) {
// 			DL_GPIO_clearInterruptStatus(MotorMonitor_E1A_PORT,
// 										 MotorMonitor_E1A_PIN);
// 			if (m1_A == m1_B) {
// 				motor1_count++;
// 			}

// 			else
// 				motor1_count--;
// 		}

// 		if (pending_pins & MotorMonitor_E1B_PIN) {
// 			DL_GPIO_clearInterruptStatus(MotorMonitor_E1B_PORT,
// 										 MotorMonitor_E1B_PIN);
// 			if (m1_A != m1_B) {
// 				motor1_count++;
// 			} else
// 				motor1_count--;
// 		}
// 	}

// 	// --- 处理电机 2 的 E2B (在 PORTA) ---
// 	if (pending_pins & MotorMonitor_E2B_PIN) {
// 		DL_GPIO_clearInterruptStatus(MotorMonitor_E2B_PORT,
// 									 MotorMonitor_E2B_PIN);

// 		// 注意：E2A 在 PORTB，需要跨端口读取它的状态
// 		bool m2_A = (DL_GPIO_readPins(MotorMonitor_E2A_PORT,
// 									  MotorMonitor_E2A_PIN) != 0);
// 		bool m2_B = (DL_GPIO_readPins(MotorMonitor_E2B_PORT,
// 									  MotorMonitor_E2B_PIN) != 0);

// 		if (m2_A != m2_B)
// 			motor2_count++;
// 		else
// 			motor2_count--;
// 	}
// }

// // ==========================================
// // GPIOB 中断服务函数 (处理 E2A)
// // ==========================================
// void GPIOB_IRQHandler(void) {
// 	// printf("中断B");
// 	uint32_t pending_pins = DL_GPIO_getPendingInterrupt(MotorMonitor_E2A_PORT);
// 	printf("pending_pins: %u, all: %d", pending_pins,
// 		   pending_pins && MotorMonitor_E2A_PIN);
// 	// --- 处理电机 2 的 E2A (在 PORTB) ---
// 	if (pending_pins & MotorMonitor_E2A_PIN) {
// 		DL_GPIO_clearInterruptStatus(MotorMonitor_E2A_PORT,
// 									 MotorMonitor_E2A_PIN);

// 		// 注意：E2B 在 PORTA，需要跨端口读取它的状态
// 		bool m2_A = (DL_GPIO_readPins(MotorMonitor_E2A_PORT,
// 									  MotorMonitor_E2A_PIN) != 0);
// 		bool m2_B = (DL_GPIO_readPins(MotorMonitor_E2B_PORT,
// 									  MotorMonitor_E2B_PIN) != 0);

// 		if (m2_A == m2_B)
// 			motor2_count++;
// 		else
// 			motor2_count--;
// 	}
// }

// MSPM0 的 GPIOA/GPIOB 外部中断属于 GROUP1 向量，
// 这里做一次分发，避免中断落入默认处理函数导致“卡死”。
void GROUP1_IRQHandler(void) {
	volatile bool m1_A, m1_B, m2_A, m2_B;
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
			motor1_count++;
		else
			motor1_count--;
	}
	if (gpioA_iidx == MotorMonitor_E1B_IIDX) {
		DL_GPIO_clearInterruptStatus(MotorMonitor_E1B_PORT,
									 MotorMonitor_E1B_PIN);
		m1_A = (DL_GPIO_readPins(MotorMonitor_E1A_PORT, MotorMonitor_E1A_PIN) !=
				0);
		m1_B = (DL_GPIO_readPins(MotorMonitor_E1B_PORT, MotorMonitor_E1B_PIN) !=
				0);
		if (m1_A != m1_B)
			motor1_count++;
		else
			motor1_count--;
	}
	if (gpioA_iidx == MotorMonitor_E2B_IIDX) {
		DL_GPIO_clearInterruptStatus(MotorMonitor_E2B_PORT,
									 MotorMonitor_E2B_PIN);
		m2_A = (DL_GPIO_readPins(MotorMonitor_E2A_PORT, MotorMonitor_E2A_PIN) !=
				0);
		m2_B = (DL_GPIO_readPins(MotorMonitor_E2B_PORT, MotorMonitor_E2B_PIN) !=
				0);
		if (m2_A != m2_B)
			motor2_count++;
		else
			motor2_count--;
	}

	// 处理 GPIOA 的中断
	// switch (gpioA_iidx) {
	// 	case MotorMonitor_E1A_IIDX:

	// 		break;

	// 	default:
	// 		break;

	// 	case MotorMonitor_E1B_IIDX:

	// 		break;

	// 	case MotorMonitor_E2B_IIDX:

	// 		break;
	// }

	// 处理 GPIOB 的中断
	switch (gpioB_iidx) {
		printf("gpioB:%d", gpioB_iidx);
	case MotorMonitor_E2A_IIDX:
		DL_GPIO_clearInterruptStatus(MotorMonitor_E2A_PORT,
									 MotorMonitor_E2A_PIN);
		m2_A = (DL_GPIO_readPins(MotorMonitor_E2A_PORT, MotorMonitor_E2A_PIN) !=
				0);
		m2_B = (DL_GPIO_readPins(MotorMonitor_E2B_PORT, MotorMonitor_E2B_PIN) !=
				0);
		if (m2_A == m2_B)
			motor2_count++;
		else
			motor2_count--;
		break;

	default:
		break;
	}
}