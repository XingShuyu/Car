#ifndef STAGE_RUNNER_H
#define STAGE_RUNNER_H

#include <stdbool.h>
#include <stdint.h>

#include "Communication/car_sync.h"
#include "Stage/Stage.h"

typedef struct StageRunner_Config {
	int base_speed_mmps;
	int round_speed_mmps;
	CarSyncRole sync_role;
	uint16_t sync_peer_address;
	uint8_t sync_run_id;
} StageRunner_Config;

void StageRunner_Init(const StageCommand *commands, int goal,
					  const StageRunner_Config *config);
bool StageRunner_Update(uint32_t nowTime);
int StageRunner_GetStageIndex(void);
int StageRunner_GetStageFlag(void);

#endif
