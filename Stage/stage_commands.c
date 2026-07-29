#include "Stage/Stage.h"

static const uint16_t armDemoPwm[][JIBOT_SERVO_COUNT] = {
	{1695,1329,1651,1283},
	{1695,1229,1851,783},
	{1695,1212,1854,784},
	{1697,1640,1869,1282},
	{2038,1689,2028,1406},
	{2035,1497,1948,904},
	{2038,1392,1957,930},
	{2035,1425,1956,930},
	{1500,1500,1500,1500},
};

static const StageArmMotionData armDemoData = {
	ARM_MOTION_SEQUENCE(armDemoPwm),
};

/* 地图 7：MaixCam 抓取测试的识别前固定待命姿态。 */
static const uint16_t armGrabTestInitialPwm[][JIBOT_SERVO_COUNT] = {
	{1500U, 1166U, 834U, 1800U},
};

static const StageArmMotionData armGrabTestInitialData = {
	ARM_MOTION_SEQUENCE(armGrabTestInitialPwm),
};

static const StageArmMaixCamGrabData armGrabTestData = {
	20.0F,
};

static const StageCommand command0[] = {
	STAGE_CMD(StageCross),
	// STAGE_CMD_NUM(StageForward, 500),
	// STAGE_CMD_NUM(StageTurn, 90.0),
	// STAGE_CMD_NUM(StageForward, 400),
	// STAGE_CMD(StageButtonContinue),
	// // STAGE_CMD_MAIXCAM(((const uint8_t[]){0x01U, 0x01U}), 2U),
	// STAGE_CMD_NUM(StageTurn, -90.0),
	// STAGE_CMD_NUM(StageForward, 380),
	// STAGE_CMD_NUM(StageTurn, -90.0),
	// STAGE_CMD_NUM(StageForward, 1000),
	// STAGE_CMD_NUM(StageTurn, -90.0),
	// //STAGE_CMD(StageButtonContinue),
	// STAGE_CMD_MAIXCAM(((const uint8_t[]){0x01U, 0x02U}), 2U),
	// STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	// STAGE_CMD(StageButtonContinue),
	// //STAGE_CMD_MAIXCAM(((const uint8_t[]){0x01U, 0x02U}), 2U),
	// STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	// STAGE_CMD(StageButtonContinue),
	// //STAGE_CMD_MAIXCAM(((const uint8_t[]){0x01U, 0x02U}), 2U),
	// STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	// STAGE_CMD(StageButtonContinue),
	// //STAGE_CMD_MAIXCAM(((const uint8_t[]){0x01U, 0x02U}), 2U),
	// STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	// STAGE_CMD(StageButtonContinue),
	// //STAGE_CMD_MAIXCAM(((const uint8_t[]){0x01U, 0x02U}), 2U),
	// STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	// STAGE_CMD(StageButtonContinue),
	// //STAGE_CMD_MAIXCAM(((const uint8_t[]){0x01U, 0x02U}), 2U),
	// STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	// STAGE_CMD(StageButtonContinue),
	// //STAGE_CMD_MAIXCAM(((const uint8_t[]){0x01U, 0x02U}), 2U),
	// STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	// STAGE_CMD(StageButtonContinue),
	// //STAGE_CMD_MAIXCAM(((const uint8_t[]){0x01U, 0x02U}), 2U),
	// STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	// STAGE_CMD(StageButtonContinue),
	// //STAGE_CMD_MAIXCAM(((const uint8_t[]){0x01U, 0x02U}), 2U),
	// // STAGE_CMD_NUM(StageTurn, -180),
	// // STAGE_CMD_NUM(StageForward, 330),
	// // STAGE_CMD_NUM(StageTurn, 90.0),
	// // STAGE_CMD_NUM(StageForward, 900),
	STAGE_CMD(StageStop),
};

static const StageCommand command1[] = {
	STAGE_CMD_NUM(StageForward, 500),
	STAGE_CMD_NUM(StageTurn, 90.0),
	STAGE_CMD_NUM(StageForward, 400),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x02U, 0x01U}), 2U),
	STAGE_CMD_NUM(StageTurn, -90.0),
	STAGE_CMD_NUM(StageForward, 480),
	STAGE_CMD_NUM(StageTurn, -90.0),
	STAGE_CMD_NUM(StageForward, 1000),
	STAGE_CMD_NUM(StageTurn, -90.0),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x02U, 0x02U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x02U, 0x02U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x02U, 0x02U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x02U, 0x02U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x02U, 0x02U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x02U, 0x02U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x02U, 0x02U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x02U, 0x02U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x02U, 0x02U}), 2U),
	// STAGE_CMD_NUM(StageTurn, -180),
	// STAGE_CMD_NUM(StageForward, 330),
	// STAGE_CMD_NUM(StageTurn, 90.0),
	// STAGE_CMD_NUM(StageForward, 900),
	STAGE_CMD(StageStop),
};

static const StageCommand command2[] = {
	STAGE_CMD_NUM(StageForward, 980),
	STAGE_CMD_NUM(StageTurn, -90.0),
	STAGE_CMD_NUM(StageForward, 600),
	STAGE_CMD_NUM(StageTurn, -90.0),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x03U, 0x01U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x03U, 0x01U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x03U, 0x01U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x03U, 0x01U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x03U, 0x01U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x03U, 0x01U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x03U, 0x01U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x03U, 0x01U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x03U, 0x01U}), 2U),
	STAGE_CMD_NUM(StageTurn, -180.0),
	STAGE_CMD_NUM(StageForward, 350),
	STAGE_CMD_NUM(StageTurn, 90.0),
	STAGE_CMD_NUM(StageForward, 980),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x03U, 0x02U}), 2U),
};

static const StageCommand command3[] = {
	STAGE_CMD_NUM(StageForward, 980),
	STAGE_CMD_NUM(StageTurn, -90.0),
	STAGE_CMD_NUM(StageForward, 600),
	STAGE_CMD_NUM(StageTurn, -90.0),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x04U, 0x01U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x04U, 0x01U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x04U, 0x01U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x04U, 0x01U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x04U, 0x01U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x04U, 0x01U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x04U, 0x01U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x04U, 0x01U}), 2U),
	STAGE_CMD_DATA(StageForward, (&(StageForwardData){110.0, 150})),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x04U, 0x01U}), 2U),
	STAGE_CMD_NUM(StageTurn, -180.0),
	STAGE_CMD_NUM(StageForward, 350),
	STAGE_CMD_NUM(StageTurn, 90.0),
	STAGE_CMD_NUM(StageForward, 980),
	STAGE_CMD_MAIXCAM(((const uint8_t[]){0x04U, 0x02U}), 2U),
};

static const StageCommand command4[] = {
	STAGE_CMD_ARM_MOTION(&armDemoData),
	STAGE_CMD(StageEnd),
};

static const StageCommand command5[] = {
	STAGE_CMD(StageZigbeeWaitStart),
	STAGE_CMD(StageCross),
	STAGE_CMD(StageZigbeeNotifyDone),
	STAGE_CMD(StageEnd),
};

/* 地图 7：机械臂先到待命位，再请求 MaixCam 坐标并移动到识别目标。 */
static const StageCommand command6[] = {
	STAGE_CMD_ARM_MOTION(&armGrabTestInitialData),
	STAGE_CMD_ARM_MAIXCAM_GRAB(&armGrabTestData),
	STAGE_CMD(StageEnd),
};

const StageCommand *const commandList[STAGE_COMMAND_LIST_COUNT] = {
	command0,
	command1,
	command2,
	command3,
	command4,
	command5,
	command6,
};
