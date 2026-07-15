#include "Motor/motor_encoder.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

static volatile int32_t motorLeftCount = 0;
static volatile int32_t motorRightCount = 0;

bool MotorEncoder_HandleGpioAInterrupt(int gpioA_iidx)
{
	bool m1_A, m1_B;

	if (gpioA_iidx != MotorMonitor_E1A_IIDX) {
		return false;
	}

	DL_GPIO_clearInterruptStatus(MotorMonitor_E1A_PORT, MotorMonitor_E1A_PIN);
	m1_A = (DL_GPIO_readPins(MotorMonitor_E1A_PORT, MotorMonitor_E1A_PIN) !=
			0);
	m1_B = (DL_GPIO_readPins(MotorMonitor_E1B_PORT, MotorMonitor_E1B_PIN) !=
			0);
	if (m1_A == m1_B)
		motorLeftCount--;
	else
		motorLeftCount++;

	return true;
}

bool MotorEncoder_HandleGpioBInterrupt(int gpioB_iidx)
{
	bool m2_A, m2_B;

	if (gpioB_iidx != MotorMonitor_E2A_IIDX) {
		return false;
	}

	DL_GPIO_clearInterruptStatus(MotorMonitor_E2A_PORT, MotorMonitor_E2A_PIN);
	m2_A = (DL_GPIO_readPins(MotorMonitor_E2A_PORT, MotorMonitor_E2A_PIN) !=
			0);
	m2_B = (DL_GPIO_readPins(MotorMonitor_E2B_PORT, MotorMonitor_E2B_PIN) !=
			0);
	if (m2_A == m2_B)
		motorRightCount++;
	else
		motorRightCount--;

	return true;
}

void MotorEncoder_ReadAndClear(int32_t *left_delta_counts,
							   int32_t *right_delta_counts)
{
	int32_t leftCountSnapshot;
	int32_t rightCountSnapshot;

	// 原子化读取并清零编码器计数，避免与中断并发导致丢脉冲
	__disable_irq();
	leftCountSnapshot = motorLeftCount;
	rightCountSnapshot = motorRightCount;
	motorLeftCount = 0;
	motorRightCount = 0;
	__enable_irq();

	if (left_delta_counts != NULL) {
		*left_delta_counts = leftCountSnapshot;
	}
	if (right_delta_counts != NULL) {
		*right_delta_counts = rightCountSnapshot;
	}
}

void MotorEncoder_ResetCounts(void)
{
	__disable_irq();
	motorLeftCount = 0;
	motorRightCount = 0;
	__enable_irq();
}
