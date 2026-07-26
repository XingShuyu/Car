#include "Motor/motor_step_test.h"

#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#include "BasicMicroLib/getTime.h"
#include "BasicMicroLib/usart.h"
#include "Communication/bluetooth_serial.h"
#include "Drivers/button_select.h"
#include "Motor/motor_runtime.h"
#include "OLED/display.h"

#define MOTOR_STEP_TEST_OLED_TEXT_SIZE 22U
#define MOTOR_STEP_TEST_DISPLAY_INTERVAL_MS 100U
#define MOTOR_STEP_TEST_PID_MESSAGE_SIZE 40U

static void MotorStepTest_ShowScreen(int target_mmps)
{
	char line[MOTOR_STEP_TEST_OLED_TEXT_SIZE];

	Display_Clear();
	Display_ShowString(0U, 0U, "MOTOR PID TEST");
	snprintf(line, sizeof(line), "Target:%d mm/s", target_mmps);
	Display_ShowString(2U, 0U, line);
	Display_ShowString(5U, 0U, "B1 EXIT");
	Display_ShowString(6U, 0U, "B2 RESTART");
}

static void MotorStepTest_ShowMeasuredSpeed(float left_mmps, float right_mmps)
{
	char line[MOTOR_STEP_TEST_OLED_TEXT_SIZE];

	snprintf(line, sizeof(line), "Speed L:%d R:%d", (int)left_mmps,
			 (int)right_mmps);
	Display_ShowString(4U, 0U, line);
}

static void MotorStepTest_Start(int target_mmps)
{
	/* 每次启动都从刹停、清零后的 PID 状态开始，便于重复观察阶跃响应。 */
	MotorRuntime_ResetAndStop(NEWMOTOR_STOP_BRAKE);
	MotorRuntime_SetTargetWheelMmps((float)target_mmps, (float)target_mmps);
}

static void MotorStepTest_SendTelemetry(float left_mmps, float right_mmps)
{
	char telemetry[24];

	/* USART_WriteAsync 原样发送 \r\n，避免 printf 的换行转换造成多余 CR。 */
	snprintf(telemetry, sizeof(telemetry), "Back:%d\r\n", (int)left_mmps);
	USART_WriteAsync(telemetry);
	snprintf(telemetry, sizeof(telemetry), "Back2:%d\r\n", (int)right_mmps);
	USART_WriteAsync(telemetry);
}

static void MotorStepTest_SendPid(void)
{
	char message[MOTOR_STEP_TEST_PID_MESSAGE_SIZE];
	float kp;
	float ki;
	float kd;

	MotorRuntime_GetPid(&kp, &ki, &kd);
	/* 协议固定保留两位小数并使用 CRLF 结尾。 */
	snprintf(message, sizeof(message), "P:%05.2f,I:%05.2f,D:%05.2f\r\n", kp,
			 ki, kd);
	USART_WriteAsync(message);
}

static bool MotorStepTest_TakePidCommand(float *kp, float *ki, float *kd)
{
	const volatile uint8_t *rxBuffer;
	uint16_t rxLength;
	char message[MOTOR_STEP_TEST_PID_MESSAGE_SIZE];
	char trailing;
	uint16_t i;

	if ((kp == NULL) || (ki == NULL) || (kd == NULL) ||
		!BluetoothSerial_GetFlag()) {
		return false;
	}

	rxBuffer = BluetoothSerial_GetBuffer();
	rxLength = BluetoothSerial_GetLength();
	if (rxLength >= sizeof(message)) {
		BluetoothSerial_ClearFlag();
		return false;
	}

	for (i = 0U; i < rxLength; i++) {
		message[i] = (char)rxBuffer[i];
	}
	message[rxLength] = '\0';
	BluetoothSerial_ClearFlag();

	if (sscanf(message, "P:%f,I:%f,D:%f%c", kp, ki, kd, &trailing) != 3) {
		return false;
	}

	return isfinite(*kp) && isfinite(*ki) && isfinite(*kd);
}

void MotorStepTest_Run(int target_mmps)
{
	uint32_t lastDisplayTime;

	MotorStepTest_ShowScreen(target_mmps);
	MotorStepTest_Start(target_mmps);
	BluetoothSerial_ClearFlag();
	MotorStepTest_SendPid();
	lastDisplayTime = getNowMs();
	ButtonSelect_ResetEvents();

	while (true) {
		ButtonSelect_Event event = ButtonSelect_TakeEvent();
		uint32_t nowMs = getNowMs();
		float leftMmps;
		float rightMmps;
		float kp;
		float ki;
		float kd;

		if (event == ButtonSelectEventNext) {
			MotorRuntime_ResetAndStop(NEWMOTOR_STOP_BRAKE);
			return;
		}

		if (event == ButtonSelectEventStart) {
			MotorStepTest_Start(target_mmps);
			MotorStepTest_ShowScreen(target_mmps);
			MotorStepTest_SendPid();
			lastDisplayTime = nowMs;
		}

		/* 保持蓝牙 DMA 收发；PID 更新后立即发送一对当前实测速度。 */
		USART_PollTx();
		if (MotorStepTest_TakePidCommand(&kp, &ki, &kd)) {
			MotorRuntime_SetPid(kp, ki, kd);
			MotorStepTest_Start(target_mmps);
			MotorStepTest_ShowScreen(target_mmps);
			MotorStepTest_SendPid();
			lastDisplayTime = nowMs;
		}
		if (MotorRuntime_Update(nowMs, getNowUs())) {
			MotorRuntime_GetMeasuredWheelMmps(&leftMmps, &rightMmps);
			MotorStepTest_SendTelemetry(leftMmps, rightMmps);
		}

		if (getTimeMs(nowMs, lastDisplayTime) >=
			MOTOR_STEP_TEST_DISPLAY_INTERVAL_MS) {
			MotorRuntime_GetMeasuredWheelMmps(&leftMmps, &rightMmps);
			MotorStepTest_ShowMeasuredSpeed(leftMmps, rightMmps);
			lastDisplayTime = nowMs;
		}
	}
}
