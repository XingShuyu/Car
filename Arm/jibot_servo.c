#include "Arm/jibot_servo.h"

#include "BasicMicroLib/usart.h"
#include "BasicMicroLib/getTime.h"
#include "ti_msp_dl_config.h"

#include <stdio.h>

#define JIBOT_SERVO_CENTER_PWM     (1500.0F)
#define JIBOT_SERVO_PWM_PER_DEGREE (2000.0F / 270.0F)
#define JIBOT_SERVO_COMMAND_SIZE   (16U)
#define JIBOT_SERVO_POSITION_REPLY_SIZE (10U)
#define JIBOT_SERVO_BATCH_COMMAND_SIZE \
	(1U + 5U + (JIBOT_SERVO_COUNT * 15U) + 1U + 1U)

static bool JibotServo_HasSentCommand = false;
static uint32_t JibotServo_LastCommandMs = 0U;

static bool JibotServo_IsValidId(uint8_t id)
{
	return (id < JIBOT_SERVO_COUNT);
}

static bool JibotServo_IsValidPwm(uint16_t positionPwm)
{
	return (positionPwm >= JIBOT_SERVO_MIN_PWM) &&
		   (positionPwm <= JIBOT_SERVO_MAX_PWM);
}

static bool JibotServo_SendCommand(const char *command)
{
	uint8_t index = 0U;

	if (command == NULL) {
		return false;
	}

	while (JibotServo_HasSentCommand &&
		getTimeMs(getNowMs(), JibotServo_LastCommandMs) <
			JIBOT_SERVO_COMMAND_INTERVAL_MS) {
		/* 保持整条指令之间至少间隔 10 ms；组命令内部不插入间隔。 */
	}

	while (command[index] != '\0') {
		USART_SendData(JibotArm_INST, (unsigned char)command[index]);
		index++;
	}
	JibotServo_LastCommandMs = getNowMs();
	JibotServo_HasSentCommand = true;

	return true;
}

static void JibotServo_FlushRxFifo(void)
{
	while (!DL_UART_Main_isRXFIFOEmpty(JibotArm_INST)) {
		(void)DL_UART_Main_receiveData(JibotArm_INST);
	}
}

static bool JibotServo_ParsePositionReply(const char *reply,
						  uint8_t expectedId,
						  uint16_t *positionPwm)
{
	uint16_t parsedId;
	uint16_t parsedPwm;
	uint8_t index;

	if ((reply == NULL) || (positionPwm == NULL) ||
		(reply[0] != '#') || (reply[4] != 'P') ||
		(reply[9] != '!')) {
		return false;
	}

	for (index = 1U; index <= 3U; index++) {
		if ((reply[index] < '0') || (reply[index] > '9')) {
			return false;
		}
	}
	for (index = 5U; index <= 8U; index++) {
		if ((reply[index] < '0') || (reply[index] > '9')) {
			return false;
		}
	}

	parsedId = (uint16_t)(((uint16_t)(reply[1] - '0') * 100U) +
						 ((uint16_t)(reply[2] - '0') * 10U) +
						 (uint16_t)(reply[3] - '0'));
	if (parsedId != expectedId) {
		return false;
	}

	parsedPwm = (uint16_t)(((uint16_t)(reply[5] - '0') * 1000U) +
						  ((uint16_t)(reply[6] - '0') * 100U) +
						  ((uint16_t)(reply[7] - '0') * 10U) +
						  (uint16_t)(reply[8] - '0'));
	*positionPwm = parsedPwm;
	return true;
}

bool JibotServo_SetAngle(uint8_t id, float angle_deg, uint16_t time_ms)
{
	float pwmValue;
	uint16_t pwm;

	if (!JibotServo_IsValidId(id) ||
		(angle_deg != angle_deg) ||
		(angle_deg < JIBOT_SERVO_MIN_ANGLE_DEG) ||
		(angle_deg > JIBOT_SERVO_MAX_ANGLE_DEG) ||
		(time_ms > JIBOT_SERVO_MAX_TIME_MS)) {
		return false;
	}

	pwmValue = JIBOT_SERVO_CENTER_PWM +
			   (angle_deg * JIBOT_SERVO_PWM_PER_DEGREE);
	/* PWM 始终为正数，+0.5F 后转换即可实现四舍五入。 */
	pwm = (uint16_t)(pwmValue + 0.5F);
	return JibotServo_SetPwm(id, pwm, time_ms);
}

bool JibotServo_SetPwm(uint8_t id, uint16_t position_pwm,
					   uint16_t time_ms)
{
	char command[JIBOT_SERVO_COMMAND_SIZE];
	int written;

	if (!JibotServo_IsValidId(id) ||
		!JibotServo_IsValidPwm(position_pwm) ||
		(time_ms > JIBOT_SERVO_MAX_TIME_MS)) {
		return false;
	}

	written = snprintf(command, sizeof(command), "#%03uP%04uT%04u!",
				   (unsigned int)id, (unsigned int)position_pwm,
				   (unsigned int)time_ms);
	if ((written < 0) || ((uint32_t)written >= sizeof(command))) {
		return false;
	}

	return JibotServo_SendCommand(command);
}

bool JibotServo_SetPwmBatch(const uint16_t position_pwm[JIBOT_SERVO_COUNT],
							uint16_t time_ms)
{
	char command[JIBOT_SERVO_BATCH_COMMAND_SIZE] = "{G0000";
	uint16_t commandLength = 6U;
	uint8_t id;
	int written;

	if ((position_pwm == NULL) || (time_ms > JIBOT_SERVO_MAX_TIME_MS)) {
		return false;
	}

	for (id = 0U; id < JIBOT_SERVO_COUNT; id++) {
		if (!JibotServo_IsValidPwm(position_pwm[id])) {
			return false;
		}
	}

	for (id = 0U; id < JIBOT_SERVO_COUNT; id++) {
		written = snprintf(&command[commandLength],
						   sizeof(command) - commandLength,
						   "#%03uP%04uT%04u!", (unsigned int)id,
						   (unsigned int)position_pwm[id],
						   (unsigned int)time_ms);
		if ((written < 0) || ((uint32_t)written >=
								 sizeof(command) - commandLength)) {
			return false;
		}
		commandLength += (uint16_t)written;
	}

	command[commandLength] = '}';
	commandLength++;
	command[commandLength] = '\0';

	return JibotServo_SendCommand(command);
}

bool JibotServo_ReleaseTorque(uint8_t id)
{
	char command[JIBOT_SERVO_COMMAND_SIZE];
	int written;

	if (!JibotServo_IsValidId(id)) {
		return false;
	}

	written = snprintf(command, sizeof(command), "#%03uPULK!",
				   (unsigned int)id);
	if ((written < 0) || ((uint32_t)written >= sizeof(command))) {
		return false;
	}

	return JibotServo_SendCommand(command);
}

bool JibotServo_RestoreTorque(uint8_t id)
{
	char command[JIBOT_SERVO_COMMAND_SIZE];
	int written;

	if (!JibotServo_IsValidId(id)) {
		return false;
	}

	written = snprintf(command, sizeof(command), "#%03uPULR!",
				   (unsigned int)id);
	if ((written < 0) || ((uint32_t)written >= sizeof(command))) {
		return false;
	}

	return JibotServo_SendCommand(command);
}

bool JibotServo_ReadPosition(uint8_t id, uint16_t *position_pwm,
					 uint16_t timeout_ms)
{
	char command[JIBOT_SERVO_COMMAND_SIZE];
	char reply[JIBOT_SERVO_POSITION_REPLY_SIZE];
	uint32_t startMs;
	uint8_t replyLength = 0U;
	int written;

	if (!JibotServo_IsValidId(id) || (position_pwm == NULL)) {
		return false;
	}

	written = snprintf(command, sizeof(command), "#%03uPRAD!",
				   (unsigned int)id);
	if ((written < 0) || ((uint32_t)written >= sizeof(command))) {
		return false;
	}

	/* 舍弃先前命令可能残留的 #OK! 等应答，仅匹配本次查询的完整位置帧。 */
	JibotServo_FlushRxFifo();
	if (!JibotServo_SendCommand(command)) {
		return false;
	}

	startMs = getNowMs();
	do {
		while (!DL_UART_Main_isRXFIFOEmpty(JibotArm_INST)) {
			char data = (char)DL_UART_Main_receiveData(JibotArm_INST);

			if (data == '#') {
				reply[0] = data;
				replyLength = 1U;
			} else if (replyLength > 0U) {
				if (replyLength < JIBOT_SERVO_POSITION_REPLY_SIZE) {
					reply[replyLength] = data;
					replyLength++;
				}

				if (replyLength == JIBOT_SERVO_POSITION_REPLY_SIZE) {
					if (JibotServo_ParsePositionReply(reply, id,
											  position_pwm)) {
						return true;
					}
					replyLength = 0U;
				}
			}
		}
	} while (getTimeMs(getNowMs(), startMs) < timeout_ms);

	return false;
}
