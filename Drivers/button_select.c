#include "Drivers/button_select.h"

#include "Stage/Stage.h"
#include "ti_msp_dl_config.h"

// 状态机下标
static volatile uint8_t TextIndex = 0;

bool ButtonSelect_HandleGpioBInterrupt(int gpioB_iidx)
{
	if (gpioB_iidx == key_PIN_B23_IIDX) {
		DL_GPIO_clearInterruptStatus(key_PORT, key_PIN_B23_PIN);
		TextIndex++;
		if (TextIndex >= STAGE_COMMAND_LIST_COUNT) {
			TextIndex = 0;
		}
		return true;
	}
	if (gpioB_iidx == key_PIN_B26_IIDX) {
		DL_GPIO_clearInterruptStatus(key_PORT, key_PIN_B26_PIN);
		return true;
	}

	return false;
}

uint8_t ButtonSelect_GetIndex(void)
{
	return TextIndex;
}

void ButtonSelect_Reset(void)
{
	TextIndex = 0;
}
