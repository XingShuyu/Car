#ifndef STAGE_H
#define STAGE_H

#include <stddef.h>

typedef enum Stage {
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
	StageForward = 16
} Stage;

typedef struct StageCommand {
	Stage type;
	const void *data;
	float numData;
} StageCommand;

typedef struct StageTurnData {
	Stage type;
	const void *data;
} StageTurnData;

#define STAGE_CMD(stage_type) { (stage_type), NULL, 0.0 }
#define STAGE_CMD_DATA(stage_type, data_ptr) { (stage_type), (data_ptr), 0.0 }
#define STAGE_CMD_NUM(stage_type, num_data){ (stage_type), NULL, (num_data) }

static const StageCommand command0[] = {
	// STAGE_CMD_NUM(StageForward, 500),STAGE_CMD(StageEnd),
};

static const StageCommand command1[] = {
	STAGE_CMD(StageStartJudge), STAGE_CMD(StageRush), STAGE_CMD(StageLeftRound),
	STAGE_CMD(StageLeft), STAGE_CMD(StageRush), STAGE_CMD(StageLeftRound),
	STAGE_CMD(StageLeft), STAGE_CMD(StageRush), STAGE_CMD(StageLeftRound),
	STAGE_CMD(StageLeft), STAGE_CMD(StageRush), STAGE_CMD(StageFake),
	STAGE_CMD(StageRightRound), STAGE_CMD(StageRight), STAGE_CMD(StageRush),
	STAGE_CMD(StageFake), STAGE_CMD(StageRightRound), STAGE_CMD(StageRight),
	STAGE_CMD(StageRush), STAGE_CMD(StageRightRound), STAGE_CMD(StageRight),
	STAGE_CMD(StageStop), STAGE_CMD(StageStop), STAGE_CMD(StageRush),
	STAGE_CMD(StageRightRound), STAGE_CMD(StageRight), STAGE_CMD(StageRush),
	STAGE_CMD(StageRightRound), STAGE_CMD(StageRight), STAGE_CMD(StageRush),
	STAGE_CMD(StageFake), STAGE_CMD(StageRightRound), STAGE_CMD(StageRight),
	STAGE_CMD(StageRush), STAGE_CMD(StageLeftRound), STAGE_CMD(StageLeft),
	STAGE_CMD(StageRush), STAGE_CMD(StageLeftRound), STAGE_CMD(StageLeft),
	STAGE_CMD(StageRush), STAGE_CMD(StageFake), STAGE_CMD(StageLeftRound),
	STAGE_CMD(StageLeft), STAGE_CMD(StageStop), STAGE_CMD(StageStop),
	STAGE_CMD(StageEnd),
};

static const StageCommand command2[] = {
	STAGE_CMD(StageStartJudge), STAGE_CMD(StageRush), STAGE_CMD(StageLeftRound),
	STAGE_CMD(StageLeft), STAGE_CMD(StageRush), STAGE_CMD(StageLeftRound),
	STAGE_CMD(StageCross), STAGE_CMD(StageRush), STAGE_CMD(StageLeft),
	STAGE_CMD(StageRush), STAGE_CMD(StageLeftRound), STAGE_CMD(StageLeft),
	STAGE_CMD(StageRush), STAGE_CMD(StageLeftRound), STAGE_CMD(StageLeft),
	STAGE_CMD(StageStop), STAGE_CMD(StageStop), STAGE_CMD(StageStop),
	STAGE_CMD(StageStop), STAGE_CMD(StageStop), STAGE_CMD(StageStop),
	STAGE_CMD(StageStop), STAGE_CMD(StageStop), STAGE_CMD(StageRush),
	STAGE_CMD(StageRightRound), STAGE_CMD(StageRight), STAGE_CMD(StageRush),
	STAGE_CMD(StageRightRound), STAGE_CMD(StageRight), STAGE_CMD(StageRush),
	STAGE_CMD(StageRightRound), STAGE_CMD(StageCross), STAGE_CMD(StageRush),
	STAGE_CMD(StageRight), STAGE_CMD(StageRush), STAGE_CMD(StageRightRound),
	STAGE_CMD(StageRight), STAGE_CMD(StageStop), STAGE_CMD(StageStop),
	STAGE_CMD(StageEnd),
};

static const StageCommand command3[] = {
	STAGE_CMD(StageRush), STAGE_CMD(StageLeftRound), STAGE_CMD(StageLeft),
	STAGE_CMD(StageRush), STAGE_CMD(StageLeftRound), STAGE_CMD(StageCross),
	STAGE_CMD(StageRush), STAGE_CMD(StageSkip), STAGE_CMD(StageLeft),
	STAGE_CMD(StageRush), STAGE_CMD(StageLeftRound), STAGE_CMD(StageLeft),
	STAGE_CMD(StageRush), STAGE_CMD(StageLeftRound), STAGE_CMD(StageLeft),
	STAGE_CMD(StageRush), STAGE_CMD(StageLeftRound), STAGE_CMD(StageLeft),
	STAGE_CMD(StageRush), STAGE_CMD_NUM(StageTurn,145), STAGE_CMD(StageFinsih),
	STAGE_CMD(StageEnd),
};

static const StageCommand *const commandList[] = {
	command0,
	command1,
	command2,
	command3,
};


#endif
