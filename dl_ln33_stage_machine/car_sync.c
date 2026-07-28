#include "dl_ln33_stage_machine/car_sync.h"

#include "BasicMicroLib/getTime.h"
#include "Communication/dl_ln33.h"

#define CARSYNC_APP_PORT             0xA0U
#define CARSYNC_PROTOCOL_MAGIC0      ((uint8_t)'C')
#define CARSYNC_PROTOCOL_MAGIC1      ((uint8_t)'S')
#define CARSYNC_PROTOCOL_VERSION     0x01U
#define CARSYNC_PAYLOAD_LENGTH       5U
#define CARSYNC_TYPE_CROSS_DONE      0x01U
#define CARSYNC_TYPE_ACK             0x02U
#define CARSYNC_DONE_INTERVAL_MS     200U
#define CARSYNC_DONE_ACK_TIMEOUT_MS  3000U

static CarSyncRole s_role = CarSyncRoleSolo;
static uint16_t s_peerAddress = 0U;
static uint8_t s_runId = CARSYNC_CROSS_RUN_ID;
static bool s_crossDoneSeen = false;
static bool s_ackSent = false;
static bool s_doneNotifyStarted = false;
static uint32_t s_doneNotifyStartMs = 0U;
static uint32_t s_lastDoneSendMs = 0U;

static void CarSync_BuildPayload(uint8_t type,
							uint8_t payload[CARSYNC_PAYLOAD_LENGTH])
{
	payload[0] = CARSYNC_PROTOCOL_MAGIC0;
	payload[1] = CARSYNC_PROTOCOL_MAGIC1;
	payload[2] = CARSYNC_PROTOCOL_VERSION;
	payload[3] = type;
	payload[4] = s_runId;
}

static bool CarSync_SendMessage(uint8_t type)
{
	uint8_t payload[CARSYNC_PAYLOAD_LENGTH];

	CarSync_BuildPayload(type, payload);
	return DLLN33_Send(s_peerAddress, CARSYNC_APP_PORT, CARSYNC_APP_PORT,
					  payload, CARSYNC_PAYLOAD_LENGTH);
}

static bool CarSync_FrameMatches(const DLLN33_Frame *frame, uint8_t type)
{
	return (frame != NULL) &&
		   (frame->remote_address == s_peerAddress) &&
		   (frame->source_port == CARSYNC_APP_PORT) &&
		   (frame->destination_port == CARSYNC_APP_PORT) &&
		   (frame->payload_length == CARSYNC_PAYLOAD_LENGTH) &&
		   (frame->payload[0] == CARSYNC_PROTOCOL_MAGIC0) &&
		   (frame->payload[1] == CARSYNC_PROTOCOL_MAGIC1) &&
		   (frame->payload[2] == CARSYNC_PROTOCOL_VERSION) &&
		   (frame->payload[3] == type) &&
		   (frame->payload[4] == s_runId);
}

static bool CarSync_ConsumeExpected(uint8_t type)
{
	DLLN33_Frame frame;
	bool matched = false;

	while (DLLN33_TryReceive(&frame)) {
		if (CarSync_FrameMatches(&frame, type)) {
			matched = true;
		}
	}

	return matched;
}

void CarSync_Init(CarSyncRole role, uint16_t peer_address, uint8_t run_id)
{
	s_role = role;
	s_peerAddress = peer_address;
	s_runId = run_id;
	s_crossDoneSeen = false;
	s_ackSent = false;
	s_doneNotifyStarted = false;
	s_doneNotifyStartMs = 0U;
	s_lastDoneSendMs = 0U;
}

void CarSync_ClearReceived(void)
{
	DLLN33_Frame frame;

	while (DLLN33_TryReceive(&frame)) {
	}
	s_crossDoneSeen = false;
	s_ackSent = false;
}

CarSyncRole CarSync_GetRole(void)
{
	return s_role;
}

uint16_t CarSync_GetLocalAddress(CarSyncRole role)
{
	if (role == CarSyncRoleLeader) {
		return CARSYNC_LEADER_ADDRESS;
	}
	if (role == CarSyncRoleFollower) {
		return CARSYNC_FOLLOWER_ADDRESS;
	}
	return 0U;
}

uint16_t CarSync_GetPeerAddress(CarSyncRole role)
{
	if (role == CarSyncRoleLeader) {
		return CARSYNC_FOLLOWER_ADDRESS;
	}
	if (role == CarSyncRoleFollower) {
		return CARSYNC_LEADER_ADDRESS;
	}
	return 0U;
}

CarSyncNotifyStatus CarSync_SendCrossDone(uint32_t now_ms)
{
	if (s_role != CarSyncRoleLeader) {
		return CarSyncNotifyAcked;
	}

	if (CarSync_ConsumeExpected(CARSYNC_TYPE_ACK)) {
		return CarSyncNotifyAcked;
	}

	if (!s_doneNotifyStarted) {
		if (!CarSync_SendMessage(CARSYNC_TYPE_CROSS_DONE)) {
			return CarSyncNotifySendFailed;
		}
		s_doneNotifyStarted = true;
		s_doneNotifyStartMs = now_ms;
		s_lastDoneSendMs = now_ms;
		return CarSyncNotifyWaiting;
	}

	if (getTimeMs(now_ms, s_doneNotifyStartMs) >=
		CARSYNC_DONE_ACK_TIMEOUT_MS) {
		return CarSyncNotifyTimedOut;
	}

	if (getTimeMs(now_ms, s_lastDoneSendMs) >= CARSYNC_DONE_INTERVAL_MS) {
		if (!CarSync_SendMessage(CARSYNC_TYPE_CROSS_DONE)) {
			return CarSyncNotifySendFailed;
		}
		s_lastDoneSendMs = now_ms;
	}

	return CarSyncNotifyWaiting;
}

bool CarSync_WaitCrossDone(uint32_t now_ms)
{
	(void)now_ms;

	if (s_role != CarSyncRoleFollower) {
		return true;
	}
	if (s_crossDoneSeen) {
		return true;
	}

	s_crossDoneSeen = CarSync_ConsumeExpected(CARSYNC_TYPE_CROSS_DONE);
	return s_crossDoneSeen;
}

bool CarSync_SendAck(void)
{
	if (s_role != CarSyncRoleFollower) {
		return true;
	}
	if (!s_crossDoneSeen) {
		return false;
	}
	if (s_ackSent) {
		return true;
	}

	s_ackSent = CarSync_SendMessage(CARSYNC_TYPE_ACK);
	return s_ackSent;
}
