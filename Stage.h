#ifndef STAGE_H
#define STAGE_H

enum Stage {
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
	StageTurn145 = 13,
	StageSkip = 14
};
int16_t command0[]={1,5, 4, 1, 5, 4, 1, 5, 4, 1, 5, 4, 12,12};
int16_t command1[]={8,1,5,4,1,5,4,1,11,5,4,1,3,2,1,3,2,1,11,3,2,12,12,1,3,2,1,3,2,1,11,3,2,1,5,4,1,5,4,1,11,5,4,12,12};
int16_t command2[]={8,1,5,4,1,5,4,1,5,6,1,4,1,5,4,12,12,12,12,12,12,12,12,1,3,2,1,3,6,1,2,1,3,2,1,3,2,12,12};
int16_t command3[]={8,1,5,4,1,5,4,1,5,6,1,14,4,1,5,4,1,5,4,1,5,4,1,13,9};
int16_t *commandList[]={command0,command1,command2,command3};


#endif