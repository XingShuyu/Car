#ifndef STAGE_H
#define STAGE_H

#include <stddef.h>
#include <stdint.h>

/*
 * 旧循迹命令的阶段类型。保留 Stage 这个 typedef，避免影响已有命令表；
 * 枚举 tag 改为 StageType，为 DL-LN33 阶段机预留 struct Stage tag。
 */
typedef enum StageType {
	StageEnd = 0,
	StageRush = 1,
	StageRight = 2,
	StageRightRound = 3,
	StageLeft = 4,
	StageLeftRound = 5,
	StageCross = 6,
	Stageultrasonic = 7,
	StageStartJudge = 8,
	StageFinsih = 9,
	StageBizz = 10,
	StageFake = 11,
	StageStop = 12,
	StageTurn = 13,
	StageSkip = 14,
	StageStandUp = 15,
	StageForward = 16,
	StageMaixCamCommand = 17,
	/* 刹停并等待 PB26（B2/Start）按下后再执行下一条命令。 */
	StageButtonContinue = 18
} Stage;

typedef struct StageCommand {
	Stage type;
	const void *data;
	float numData;
} StageCommand;

typedef struct StageForwardData {
	float length;
	int32_t speed;
} StageForwardData;

typedef struct StageMaixCamCommandData {
	const uint8_t *bytes;
	uint16_t length;
} StageMaixCamCommandData;

#define STAGE_CMD(stage_type) {(stage_type), NULL, 0.0}
#define STAGE_CMD_DATA(stage_type, data_ptr) {(stage_type), (data_ptr), 0.0}
#define STAGE_CMD_NUM(stage_type, num_data) {(stage_type), NULL, (num_data)}
#define STAGE_CMD_MAIXCAM(data_ptr, data_len) \
	{StageMaixCamCommand, (&(StageMaixCamCommandData){(data_ptr), (data_len)}), 0.0}

#define STAGE_COMMAND_LIST_COUNT 4U

extern const StageCommand *const commandList[STAGE_COMMAND_LIST_COUNT];

#endif
