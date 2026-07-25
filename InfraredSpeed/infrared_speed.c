/**
 * @file infrared_speed.c
 * @brief 对射式红外测速门的中断时间戳采集。
 */

#include "InfraredSpeed/infrared_speed.h"

#include "BasicMicroLib/getTime.h"
#include "ti_msp_dl_config.h"

/* 小球不可能在 1 ms 内经过 30 cm；该限制用于滤除毛刺。 */
#define INFRARED_SPEED_MIN_TRAVEL_US 1000UL

static volatile InfraredSpeed_Status s_status = InfraredSpeedStatusArmed;
static volatile InfraredSpeed_Gate s_firstGate = InfraredSpeedGateA;
static volatile InfraredSpeed_Gate s_secondGate = InfraredSpeedGateB;
static volatile uint32_t s_startUs = 0U;
static volatile uint32_t s_elapsedUs = 0U;

static bool InfraredSpeed_IsGateActive(InfraredSpeed_Gate gate)
{
	uint32_t pinMask;

	if (gate == InfraredSpeedGateA) {
		pinMask = InfraredSpeed_GATE_A_PIN;
	} else {
		pinMask = InfraredSpeed_GATE_B_PIN;
	}

#if INFRARED_SPEED_ACTIVE_LOW
	return DL_GPIO_readPins(InfraredSpeed_PORT, pinMask) == 0U;
#else
	return DL_GPIO_readPins(InfraredSpeed_PORT, pinMask) != 0U;
#endif
}

static void InfraredSpeed_OnGateActive(InfraredSpeed_Gate gate, uint32_t nowUs)
{
	uint32_t elapsedUs;

	if (s_status == InfraredSpeedStatusArmed) {
		s_firstGate = gate;
		s_startUs = nowUs;
		s_status = InfraredSpeedStatusWaitingSecondGate;
		return;
	}

	if ((s_status != InfraredSpeedStatusWaitingSecondGate) ||
		(gate == s_firstGate)) {
		return;
	}

	elapsedUs = getTimeUs(nowUs, s_startUs);
	if (elapsedUs < INFRARED_SPEED_MIN_TRAVEL_US) {
		return;
	}

	s_secondGate = gate;
	s_elapsedUs = elapsedUs;
	s_status = InfraredSpeedStatusComplete;
}

void InfraredSpeed_Init(void)
{
	InfraredSpeed_Rearm();
}

void InfraredSpeed_Rearm(void)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	s_firstGate = InfraredSpeedGateA;
	s_secondGate = InfraredSpeedGateB;
	s_startUs = 0U;
	s_elapsedUs = 0U;
	s_status = InfraredSpeedStatusArmed;
	if ((primask & 1U) == 0U) {
		__enable_irq();
	}
}

void InfraredSpeed_Update(uint32_t nowUs)
{
	if ((s_status == InfraredSpeedStatusWaitingSecondGate) &&
		(getTimeUs(nowUs, s_startUs) >= INFRARED_SPEED_TIMEOUT_US)) {
		s_status = InfraredSpeedStatusTimeout;
	}
}

bool InfraredSpeed_HandleGpioBInterrupt(int gpioB_iidx)
{
	InfraredSpeed_Gate gate;

	if (gpioB_iidx == InfraredSpeed_GATE_A_IIDX) {
		gate = InfraredSpeedGateA;
		DL_GPIO_clearInterruptStatus(InfraredSpeed_PORT,
							 InfraredSpeed_GATE_A_PIN);
	} else if (gpioB_iidx == InfraredSpeed_GATE_B_IIDX) {
		gate = InfraredSpeedGateB;
		DL_GPIO_clearInterruptStatus(InfraredSpeed_PORT,
							 InfraredSpeed_GATE_B_PIN);
	} else {
		return false;
	}

	/* 双边沿中断中仅接受“被小球遮挡”的有效电平。 */
	if (InfraredSpeed_IsGateActive(gate)) {
		InfraredSpeed_OnGateActive(gate, getNowUs());
	}

	return true;
}

InfraredSpeed_Status InfraredSpeed_GetStatus(void)
{
	return s_status;
}

InfraredSpeed_Gate InfraredSpeed_GetFirstGate(void)
{
	return s_firstGate;
}

bool InfraredSpeed_GetResult(InfraredSpeed_Result *result)
{
	uint32_t elapsedUs;

	if ((result == NULL) || (s_status != InfraredSpeedStatusComplete)) {
		return false;
	}

	/* Complete 后 ISR 不会再修改结果，读取这组字段是稳定的。 */
	elapsedUs = s_elapsedUs;
	if (elapsedUs == 0U) {
		return false;
	}

	result->firstGate = s_firstGate;
	result->secondGate = s_secondGate;
	result->elapsedUs = elapsedUs;
	result->speedMps =
		((float)INFRARED_SPEED_GATE_DISTANCE_MM * 1000.0F) /
		(float)elapsedUs;
	return true;
}
