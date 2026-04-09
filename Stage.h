#ifndef STAGE_H
#define STAGE_H

enum Stage {
	StageStart = 1,
	StageRight = 2,
    StageRightRound =3,
	StageLeft = 4,
    StageLeftRound = 5,
	StageCross = 6,
	Stageultrasonic=7,
	StageBack=8,
	StageFinsih=9,
	StageBizz=10
};

int16_t command[] = {
    1,2,1,3,6,10,1,4,5,4,5,2,3,6,5,7,8,6,5,7,8,6,5,7,8,9
	// 2,9
};

#endif