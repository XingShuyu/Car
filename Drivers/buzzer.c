#include "Drivers/buzzer.h"

#include "BasicMicroLib/delay.h"
#include "ti_msp_dl_config.h"

// 蜂鸣器鸣响三声
void Buzzer_Beep(void)
{
	for (int i = 0; i < 3; i++) {
		DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_16); // 打开蜂鸣器
		delay_ms(100);
		DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_16); // 关闭蜂鸣器
		delay_ms(100);
	}
}
