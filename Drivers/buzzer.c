#include "Drivers/buzzer.h"

#include <stdbool.h>

#include "BasicMicroLib/delay.h"
#include "ti_msp_dl_config.h"

#define BUZZER_SINGLE_BEEP_DURATION_MS 100U

static bool buzzerSingleBeepActive = false;
static uint32_t buzzerSingleBeepStartMs = 0U;

// 蜂鸣器鸣响三声
void Buzzer_Beep(void)
{
	buzzerSingleBeepActive = false;
	for (int i = 0; i < 3; i++) {
		DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_16); // 打开蜂鸣器
		delay_ms(100);
		DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_16); // 关闭蜂鸣器
		delay_ms(100);
	}
}

void Buzzer_BeepOnceAsync(uint32_t nowMs)
{
	DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_16);
	buzzerSingleBeepStartMs = nowMs;
	buzzerSingleBeepActive = true;
}

void Buzzer_Update(uint32_t nowMs)
{
	if (buzzerSingleBeepActive &&
		((uint32_t)(nowMs - buzzerSingleBeepStartMs) >=
		 BUZZER_SINGLE_BEEP_DURATION_MS)) {
		DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_16);
		buzzerSingleBeepActive = false;
	}
}
