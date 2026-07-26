#ifndef STAGE_H
#define STAGE_H

#include <stddef.h>
#include <stdint.h>

#include "Arm/arm_ik_motion.h"
#include "Arm/arm_motion_state.h"

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
	StageButtonContinue = 18,
	/* 六轴按 PWM 动作表逐列运动；完成后自动执行下一条阶段命令。 */
	StageArmMotion = 19,
	/* 自动优先钩爪朝下的三参数笛卡尔逆解动作；完成后执行下一条命令。 */
	StageArmIkMotion = 20,
	/* Follower 刹停等待 Leader 的 StageCross 完成消息。 */
	StageZigbeeWaitStart = 21,
	/* Leader 在 StageCross 完成后通知 Follower。 */
	StageZigbeeNotifyDone = 22
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

typedef struct StageArmMotionData {
	ArmMotionState_Sequence sequence;
} StageArmMotionData;

/** yaw(度)、x/y(mm) 的自动优先钩爪朝下笛卡尔动作参数。 */
typedef struct StageArmIkMotionData {
	float yawDeg;
	float xMm;
	float yMm;
} StageArmIkMotionData;

/*
 * 使用示例（每一行是一个动作，第二维固定为六个舵机 ID）：
 *
 * static const uint16_t armDemoPwm[][JIBOT_SERVO_COUNT] = {
 *     {1500U, 1500U, 1500U, 1500U, 1500U, 1500U}, // 动作 0，ID 0~5
 *     {1600U, 1500U, 1400U, 1500U, 1500U, 1800U}, // 动作 1
 *     {1700U, 1500U, 1300U, 1500U, 1500U, 1500U}, // 动作 2
 * };
 * static const StageArmMotionData armDemoData = {
 *     ARM_MOTION_SEQUENCE(armDemoPwm),
 * };
 * static const StageCommand command[] = {
 *     STAGE_CMD_ARM_MOTION(&armDemoData),
 *     STAGE_CMD(StageEnd),
 * };
 */

#define STAGE_CMD(stage_type) {(stage_type), NULL, 0.0}
#define STAGE_CMD_DATA(stage_type, data_ptr) {(stage_type), (data_ptr), 0.0}
#define STAGE_CMD_NUM(stage_type, num_data) {(stage_type), NULL, (num_data)}
#define STAGE_CMD_MAIXCAM(data_ptr, data_len) \
	{StageMaixCamCommand, (&(StageMaixCamCommandData){(data_ptr), (data_len)}), 0.0}
#define STAGE_CMD_ARM_MOTION(data_ptr) \
	{StageArmMotion, (data_ptr), 0.0}
#define STAGE_CMD_ARM_IK_MOTION(data_ptr) \
	{StageArmIkMotion, (data_ptr), 0.0}

#define STAGE_COMMAND_LIST_COUNT 6U

extern const StageCommand *const commandList[STAGE_COMMAND_LIST_COUNT];

#endif
