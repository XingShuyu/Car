#include "dl_ln33_stage_machine/dl_ln33_stage_machine.h"

#include <stddef.h>

static bool StageMachine_MessageIsValid(const StageMessage *message)
{
	return (message != NULL) &&
		   (message->payload_length <= DLLN33_MAX_PAYLOAD_LENGTH) &&
		   ((message->payload_length == 0U) || (message->payload != NULL));
}

static bool StageMachine_ConfigurationIsValid(const struct Stage *stages,
												 uint16_t stage_count)
{
	uint16_t index;

	if ((stages == NULL) || (stage_count == 0U)) {
		return false;
	}

	for (index = 0U; index < stage_count; index++) {
		if (!StageMachine_MessageIsValid(stages[index].finish_message)) {
			return false;
		}
		if (stages[index].enter_message != NULL) {
			if (!StageMachine_MessageIsValid(stages[index].enter_message) ||
				(stages[index].enter_message->source_port <
				 DLLN33_APPLICATION_PORT_MIN)) {
				return false;
			}
		}
	}

	return true;
}

static bool StageMachine_SendEnterMessage(const StageMessage *message)
{
	if (message == NULL) {
		return true;
	}

	return DLLN33_Send(message->remote_address, message->source_port,
					message->destination_port, message->payload,
					message->payload_length);
}

static bool StageMachine_IsFinishMessage(const StageMessage *expected,
											const DLLN33_Frame *received)
{
	uint8_t index;

	if ((expected == NULL) || (received == NULL) ||
		((expected->remote_address != STAGE_MESSAGE_ANY_REMOTE_ADDRESS) &&
		 (expected->remote_address != received->remote_address)) ||
		(expected->source_port != received->source_port) ||
		(expected->destination_port != received->destination_port) ||
		(expected->payload_length != received->payload_length)) {
		return false;
	}

	for (index = 0U; index < expected->payload_length; index++) {
		if (expected->payload[index] != received->payload[index]) {
			return false;
		}
	}

	return true;
}

bool StageMachine_Init(StageMachine *machine, const struct Stage *stages,
				   uint16_t stage_count, void *user_context)
{
	if (machine == NULL) {
		return false;
	}

	machine->stages = stages;
	machine->stage_count = stage_count;
	machine->stage_index = 0U;
	machine->user_context = user_context;
	machine->status = StageMachineStatus_InvalidConfiguration;

	if (!StageMachine_ConfigurationIsValid(stages, stage_count)) {
		return false;
	}

	machine->status = StageMachineStatus_Entering;
	return true;
}

void StageMachine_Update(StageMachine *machine)
{
	const struct Stage *stage;
	DLLN33_Frame received;

	if ((machine == NULL) ||
		(machine->status == StageMachineStatus_Uninitialized) ||
		(machine->status == StageMachineStatus_InvalidConfiguration) ||
		(machine->status == StageMachineStatus_Complete)) {
		return;
	}

	stage = &machine->stages[machine->stage_index];
	if (machine->status == StageMachineStatus_Entering) {
		if (!StageMachine_SendEnterMessage(stage->enter_message)) {
			return;
		}
		machine->status = StageMachineStatus_WaitingForFinish;
	}

	while (DLLN33_TryReceive(&received)) {
		if (StageMachine_IsFinishMessage(stage->finish_message, &received)) {
			machine->stage_index++;
			if (machine->stage_index >= machine->stage_count) {
				machine->status = StageMachineStatus_Complete;
			} else {
				/* 下一阶段先发送 enter_message，再处理它的结束消息。 */
				machine->status = StageMachineStatus_Entering;
			}
			return;
		}

		if (stage->non_finish_message_handler != NULL) {
			stage->non_finish_message_handler(&received, machine->user_context);
		}
	}
}

StageMachineStatus StageMachine_GetStatus(const StageMachine *machine)
{
	if (machine == NULL) {
		return StageMachineStatus_InvalidConfiguration;
	}
	return machine->status;
}

uint16_t StageMachine_GetStageIndex(const StageMachine *machine)
{
	if (machine == NULL) {
		return 0U;
	}
	return machine->stage_index;
}
