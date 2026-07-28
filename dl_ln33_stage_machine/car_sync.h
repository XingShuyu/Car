/* Project-root multi-car synchronization protocol for stage commands. */
#ifndef CAR_SYNC_H
#define CAR_SYNC_H

#include <stdbool.h>
#include <stdint.h>

#define CARSYNC_LEADER_ADDRESS   0x0001U
#define CARSYNC_FOLLOWER_ADDRESS 0x0002U
#define CARSYNC_NETWORK_ID       0x2530U
#define CARSYNC_CHANNEL          0x0FU
#define CARSYNC_CROSS_RUN_ID     0x01U

/* Runtime role selected from the boot menu for the StageCross relay route. */
typedef enum CarSyncRole {
	CarSyncRoleSolo = 0,
	CarSyncRoleLeader,
	CarSyncRoleFollower,
} CarSyncRole;

/* Leader waits for ACK, but falls through on timeout so the route can end. */
typedef enum CarSyncNotifyStatus {
	CarSyncNotifyWaiting = 0,
	CarSyncNotifyAcked,
	CarSyncNotifyTimedOut,
	CarSyncNotifySendFailed,
} CarSyncNotifyStatus;

void CarSync_Init(CarSyncRole role, uint16_t peer_address, uint8_t run_id);
void CarSync_ClearReceived(void);
CarSyncRole CarSync_GetRole(void);
uint16_t CarSync_GetLocalAddress(CarSyncRole role);
uint16_t CarSync_GetPeerAddress(CarSyncRole role);
CarSyncNotifyStatus CarSync_SendCrossDone(uint32_t now_ms);
bool CarSync_WaitCrossDone(uint32_t now_ms);
bool CarSync_SendAck(void);

#endif
