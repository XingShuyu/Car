/**
 * @file infrared_speed_test.c
 * @brief 对射式红外测速门的 OLED 测试界面。
 */

#include "InfraredSpeed/infrared_speed_test.h"

#include "BasicMicroLib/getTime.h"
#include "BasicMicroLib/usart.h"
#include "Communication/maixcam_serial.h"
#include "Drivers/button_select.h"
#include "InfraredSpeed/infrared_speed.h"
#include "OLED/display.h"
#include <stdio.h>

static char InfraredSpeedTest_GateName(InfraredSpeed_Gate gate)
{
	return (gate == InfraredSpeedGateA) ? 'A' : 'B';
}

static void InfraredSpeedTest_ShowArmed(void)
{
	Display_Clear();
	Display_ShowString(0U, 0U, "IR SPEED TEST");
	Display_ShowString(2U, 0U, "DIST: 0.30 m");
	Display_ShowString(4U, 0U, "PASS TWO GATES");
	Display_ShowString(6U, 0U, "B1 REARM B2 EXIT");
}

static void InfraredSpeedTest_ShowWaiting(void)
{
	char line[21];

	snprintf(line, sizeof(line), "GATE %c FIRST",
			 InfraredSpeedTest_GateName(InfraredSpeed_GetFirstGate()));
	Display_Clear();
	Display_ShowString(0U, 0U, "MEASURING...");
	Display_ShowString(2U, 0U, line);
	Display_ShowString(4U, 0U, "PASS OTHER GATE");
	Display_ShowString(6U, 0U, "B1 REARM B2 EXIT");
}

static void InfraredSpeedTest_ShowResult(void)
{
	InfraredSpeed_Result result;
	char line[21];

	if (!InfraredSpeed_GetResult(&result)) {
		return;
	}

	Display_Clear();
	snprintf(line, sizeof(line), "V: %.3f m/s", result.speedMps);
	Display_ShowString(0U, 0U, line);
	snprintf(line, sizeof(line), "T: %.3f ms",
			 (float)result.elapsedUs / 1000.0F);
	Display_ShowString(2U, 0U, line);
	snprintf(line, sizeof(line), "%c -> %c  (0.30m)",
			 InfraredSpeedTest_GateName(result.firstGate),
			 InfraredSpeedTest_GateName(result.secondGate));
	Display_ShowString(4U, 0U, line);
	Display_ShowString(6U, 0U, "B1 REARM B2 EXIT");
}

static void InfraredSpeedTest_ShowTimeout(void)
{
	Display_Clear();
	Display_ShowString(0U, 0U, "MEASURE TIMEOUT");
	Display_ShowString(2U, 0U, "TRY AGAIN");
	Display_ShowString(6U, 0U, "B1 REARM B2 EXIT");
}

void InfraredSpeedTest_Run(void)
{
	InfraredSpeed_Status shownStatus = InfraredSpeedStatusTimeout;

	ButtonSelect_ResetEvents();
	InfraredSpeed_Rearm();

	while (true) {
		ButtonSelect_Event event;
		InfraredSpeed_Status status;

		InfraredSpeed_Update(getNowUs());
		status = InfraredSpeed_GetStatus();
		if (status != shownStatus) {
			shownStatus = status;
			if (status == InfraredSpeedStatusArmed) {
				InfraredSpeedTest_ShowArmed();
			} else if (status == InfraredSpeedStatusWaitingSecondGate) {
				InfraredSpeedTest_ShowWaiting();
			} else if (status == InfraredSpeedStatusComplete) {
				InfraredSpeedTest_ShowResult();
			} else {
				InfraredSpeedTest_ShowTimeout();
			}
		}

		event = ButtonSelect_TakeEvent();
		if (event == ButtonSelectEventNext) {
			InfraredSpeed_Rearm();
		} else if (event == ButtonSelectEventStart) {
			return;
		}

		/* 测试期间仍维持串口收发，避免通信状态堆积。 */
		USART_PollTx();
		MaixCamSerial_Poll();
	}
}
