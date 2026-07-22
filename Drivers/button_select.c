#include "Drivers/button_select.h"

#include "BasicMicroLib/getTime.h"
#include "ti_msp_dl_config.h"

#define BUTTON_SELECT_DEBOUNCE_MS 80U

/* 中断只置位事件；路线和菜单状态不放入驱动层。 */
static volatile bool nextEventPending = false;
static volatile bool startEventPending = false;
static volatile uint32_t lastNextEventMs = 0U;
static volatile uint32_t lastStartEventMs = 0U;

bool ButtonSelect_HandleGpioBInterrupt(int gpioB_iidx)
{
	uint32_t nowMs;

	if (gpioB_iidx == key_PIN_B23_IIDX) {
		DL_GPIO_clearInterruptStatus(key_PORT, key_PIN_B23_PIN);
		nowMs = getNowMs();
		if (getTimeMs(nowMs, lastNextEventMs) >=
			BUTTON_SELECT_DEBOUNCE_MS) {
			lastNextEventMs = nowMs;
			nextEventPending = true;
		}
		return true;
	}
	if (gpioB_iidx == key_PIN_B26_IIDX) {
		DL_GPIO_clearInterruptStatus(key_PORT, key_PIN_B26_PIN);
		nowMs = getNowMs();
		if (getTimeMs(nowMs, lastStartEventMs) >=
			BUTTON_SELECT_DEBOUNCE_MS) {
			lastStartEventMs = nowMs;
			startEventPending = true;
		}
		return true;
	}

	return false;
}

ButtonSelect_Event ButtonSelect_TakeEvent(void)
{
	if (nextEventPending) {
		nextEventPending = false;
		return ButtonSelectEventNext;
	}

	if (startEventPending) {
		startEventPending = false;
		return ButtonSelectEventStart;
	}

	return ButtonSelectEventNone;
}

void ButtonSelect_ResetEvents(void)
{
	uint32_t nowMs = getNowMs();

	nextEventPending = false;
	startEventPending = false;
	/* 进入菜单后第一下按键可以立即生效。 */
	lastNextEventMs = nowMs - BUTTON_SELECT_DEBOUNCE_MS;
	lastStartEventMs = nowMs - BUTTON_SELECT_DEBOUNCE_MS;
}
