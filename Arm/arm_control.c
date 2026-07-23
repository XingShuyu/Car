#include "Arm/arm_control.h"

#include "Drivers/button_select.h"
#include "OLED/display.h"

#include <stdio.h>

#define ARM_CONTROL_DISPLAY_LINE_SIZE (22U)

static bool ArmControl_ReleaseAllTorque(void)
{
	bool allSucceeded = true;
	uint8_t id;

	for (id = 0U; id < JIBOT_SERVO_COUNT; id++) {
		if (!JibotServo_ReleaseTorque(id)) {
			allSucceeded = false;
		}
	}

	return allSucceeded;
}

static bool ArmControl_RestoreAllTorque(void)
{
	bool allSucceeded = true;
	uint8_t id;

	for (id = 0U; id < JIBOT_SERVO_COUNT; id++) {
		if (!JibotServo_RestoreTorque(id)) {
			allSucceeded = false;
		}
	}

	return allSucceeded;
}

static void ArmControl_ShowTeachReleased(void)
{
	Display_Clear();
	Display_ShowString(0U, 0U, "ARM TEACH: FREE");
	Display_ShowString(2U, 0U, "Move arm by hand");
	Display_ShowString(5U, 0U, "B2 SAVE & LOCK");
	Display_ShowString(6U, 0U, "B1 EXIT");
}

static void ArmControl_ShowPwmValue(uint8_t row, uint8_t firstId,
					 const ArmControl_PwmState *state)
{
	char line[ARM_CONTROL_DISPLAY_LINE_SIZE];
	const char *firstValue = "----";
	const char *secondValue = "----";
	char firstBuffer[6];
	char secondBuffer[6];

	if ((state->validMask & (uint8_t)(1U << firstId)) != 0U) {
		snprintf(firstBuffer, sizeof(firstBuffer), "%04u",
			 (unsigned int)state->pwm[firstId]);
		firstValue = firstBuffer;
	}
	if ((state->validMask & (uint8_t)(1U << (firstId + 1U))) != 0U) {
		snprintf(secondBuffer, sizeof(secondBuffer), "%04u",
			 (unsigned int)state->pwm[firstId + 1U]);
		secondValue = secondBuffer;
	}

	snprintf(line, sizeof(line), "%u:%s %u:%s", (unsigned int)firstId,
		 firstValue, (unsigned int)(firstId + 1U),
		 secondValue);
	Display_ShowString(row, 0U, line);
}

static void ArmControl_ShowTeachCaptured(const ArmControl_PwmState *state,
						 bool allRead)
{
	Display_Clear();
	Display_ShowString(0U, 0U, allRead ? "PWM SAVED & LOCKED" :
								"PWM READ PARTIAL");
	ArmControl_ShowPwmValue(1U, 0U, state);
	ArmControl_ShowPwmValue(2U, 2U, state);
	ArmControl_ShowPwmValue(3U, 4U, state);
	Display_ShowString(5U, 0U, "B2 RELEASE");
	Display_ShowString(6U, 0U, "B1 EXIT");
}

bool ArmControl_ReadAllPwm(ArmControl_PwmState *state, uint16_t timeout_ms)
{
	bool allSucceeded = true;
	uint8_t id;

	if (state == NULL) {
		return false;
	}

	state->validMask = 0U;
	for (id = 0U; id < JIBOT_SERVO_COUNT; id++) {
		state->pwm[id] = 0U;
		if (JibotServo_ReadPosition(id, &state->pwm[id], timeout_ms)) {
			state->validMask |= (uint8_t)(1U << id);
		} else {
			allSucceeded = false;
		}
	}

	return allSucceeded;
}

void ArmControl_RunTeachTest(void)
{
	ArmControl_PwmState state;
	bool torqueReleased;

	/* 进入示教模式即释放扭力；退出路径始终恢复扭力。 */
	torqueReleased = ArmControl_ReleaseAllTorque();
	ArmControl_ShowTeachReleased();
	if (!torqueReleased) {
		Display_ShowString(7U, 0U, "RELEASE CMD FAIL");
	}

	ButtonSelect_ResetEvents();
	while (true) {
		ButtonSelect_Event event = ButtonSelect_TakeEvent();

		if (event == ButtonSelectEventNext) {
			(void)ArmControl_RestoreAllTorque();
			Display_Clear();
			Display_ShowString(2U, 0U, "ARM TEST EXIT");
			return;
		}

		if (event != ButtonSelectEventStart) {
			continue;
		}

		if (torqueReleased) {
			bool allRead;

			Display_Clear();
			Display_ShowString(2U, 0U, "READING PWM...");
			allRead = ArmControl_ReadAllPwm(&state,
									 ARM_CONTROL_READ_TIMEOUT_MS);
			(void)ArmControl_RestoreAllTorque();
			torqueReleased = false;
			ArmControl_ShowTeachCaptured(&state, allRead);
		} else {
			torqueReleased = ArmControl_ReleaseAllTorque();
			ArmControl_ShowTeachReleased();
			if (!torqueReleased) {
				Display_ShowString(7U, 0U, "RELEASE CMD FAIL");
			}
		}
	}
}
