#ifndef STAGE_RUNNER_H
#define STAGE_RUNNER_H

#include <stdbool.h>
#include <stdint.h>

#include "Stage/Stage.h"

typedef struct StageRunner_Config {
	int base_speed_mmps;
	int round_speed_mmps;
} StageRunner_Config;

void StageRunner_Init(const StageCommand *commands, int goal,
					  const StageRunner_Config *config);
static void StageRunner_SendMaixCamStageNotify(uint8_t GimbalStage);
bool StageRunner_Update(uint32_t nowTime);
int StageRunner_GetStageIndex(void);
int StageRunner_GetStageFlag(void);

#endif
