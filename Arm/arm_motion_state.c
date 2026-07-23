#include "Arm/arm_motion_state.h"

#include "BasicMicroLib/getTime.h"

static bool ArmMotionState_SendFrame(ArmMotionState *state)
{
	uint16_t positionPwm[JIBOT_SERVO_COUNT];
	uint8_t id;

	for (id = 0U; id < JIBOT_SERVO_COUNT; id++) {
		positionPwm[id] = state->sequence.pwmData[
			(uint32_t)state->frameIndex * JIBOT_SERVO_COUNT + id];
	}

	return JibotServo_SetPwmBatch(positionPwm,
							 ARM_MOTION_STATE_MOVE_TIME_MS);
}

static bool ArmMotionState_IsCurrentFrameReached(
	const ArmMotionState *state)
{
	bool allReached = true;
	uint8_t id;

	for (id = 0U; id < JIBOT_SERVO_COUNT; id++) {
		uint16_t currentPwm;
		uint16_t targetPwm = state->sequence.pwmData[
			(uint32_t)state->frameIndex * JIBOT_SERVO_COUNT + id];
		uint16_t difference;

		/* 即使某一轴查询失败，仍继续读取其余关节的当前位置。 */
		if (!JibotServo_ReadPosition(id, &currentPwm,
									 ARM_MOTION_STATE_READ_TIMEOUT_MS)) {
			allReached = false;
			continue;
		}

		difference = (currentPwm >= targetPwm) ?
					 (uint16_t)(currentPwm - targetPwm) :
					 (uint16_t)(targetPwm - currentPwm);
		if (difference > ARM_MOTION_STATE_PWM_TOLERANCE) {
			allReached = false;
		}
	}

	return allReached;
}

void ArmMotionState_Reset(ArmMotionState *state)
{
	if (state == NULL) {
		return;
	}

	state->sequence.pwmData = NULL;
	state->sequence.frameCount = 0U;
	state->frameIndex = 0U;
	state->lastPollMs = 0U;
	state->status = ArmMotionStateIdle;
}

bool ArmMotionState_Start(ArmMotionState *state,
					  const ArmMotionState_Sequence *sequence,
					  uint32_t now_ms)
{
	if ((state == NULL) || (sequence == NULL) ||
		(sequence->pwmData == NULL) || (sequence->frameCount == 0U)) {
		if (state != NULL) {
			ArmMotionState_Reset(state);
			state->status = ArmMotionStateFailed;
		}
		return false;
	}

	state->sequence = *sequence;
	state->frameIndex = 0U;
	state->lastPollMs = now_ms;
	state->status = ArmMotionStateRunning;
	if (!ArmMotionState_SendFrame(state)) {
		state->status = ArmMotionStateFailed;
		return false;
	}
	state->lastPollMs = getNowMs();

	return true;
}

ArmMotionState_Status ArmMotionState_Update(ArmMotionState *state,
									 uint32_t now_ms)
{
	if (state == NULL) {
		return ArmMotionStateFailed;
	}
	if (state->status != ArmMotionStateRunning) {
		return state->status;
	}
	if (getTimeMs(now_ms, state->lastPollMs) <
		ARM_MOTION_STATE_STATUS_POLL_INTERVAL_MS) {
		return ArmMotionStateRunning;
	}

	if (!ArmMotionState_IsCurrentFrameReached(state)) {
		/* 查询完成后重新计时，下一轮在 500 ms 后读取并可能重发。 */
		state->lastPollMs = getNowMs();
		if (!ArmMotionState_SendFrame(state)) {
			state->status = ArmMotionStateFailed;
		} else {
			state->lastPollMs = getNowMs();
		}
		return state->status;
	}

	state->frameIndex++;
	if (state->frameIndex >= state->sequence.frameCount) {
		state->status = ArmMotionStateCompleted;
		return state->status;
	}

	if (!ArmMotionState_SendFrame(state)) {
		state->status = ArmMotionStateFailed;
	} else {
		state->lastPollMs = getNowMs();
	}

	return state->status;
}
