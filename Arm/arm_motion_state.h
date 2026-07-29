#ifndef ARM_ARM_MOTION_STATE_H
#define ARM_ARM_MOTION_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "Arm/jibot_servo.h"

/** 下发一帧目标 PWM 时写入协议的到位时间。 */
#define ARM_MOTION_STATE_MOVE_TIME_MS (1000U)

/** 当前帧未达到目标时，读取全部舵机位置的轮询间隔。 */
#define ARM_MOTION_STATE_STATUS_POLL_INTERVAL_MS (500U)

/** 单个舵机位置查询的最长等待时间。 */
#define ARM_MOTION_STATE_READ_TIMEOUT_MS (10U)

/** 当前 PWM 与目标 PWM 的允许绝对误差，可按实机精度调整。 */
#define ARM_MOTION_STATE_PWM_TOLERANCE (50U)

/**
 * 动作表描述。
 *
 * pwmData 指向一个二维数组 pwm[frameCount][JIBOT_SERVO_COUNT] 的首元素。
 * 第一维为动作序号，第二维固定为四个关节 ID；即
 * pwmData[frameIndex * JIBOT_SERVO_COUNT + id] 是第 frameIndex 行中 ID
 * 为 id 的目标 PWM。
 */
typedef struct ArmMotionState_Sequence {
	const uint16_t *pwmData;
	uint16_t frameCount;
} ArmMotionState_Sequence;

/** 将 pwm[动作数][JIBOT_SERVO_COUNT] 二维数组包装成动作表描述。 */
#define ARM_MOTION_SEQUENCE(pwm_table)                                         \
	{&(pwm_table)[0][0], (uint16_t)(sizeof(pwm_table) / sizeof((pwm_table)[0]))}

typedef enum ArmMotionState_Status {
	ArmMotionStateIdle = 0,
	ArmMotionStateRunning,
	ArmMotionStateCompleted,
	ArmMotionStateFailed,
} ArmMotionState_Status;

/** 非阻塞四轴动作状态机的运行数据。 */
typedef struct ArmMotionState {
	ArmMotionState_Sequence sequence;
	uint16_t frameIndex;
	uint32_t lastPollMs;
	ArmMotionState_Status status;
} ArmMotionState;

/**
 * @brief 初始化并立即下发动作表的第 0 列。
 *
 * 每一行会向四个舵机同时下发原始 PWM 位置命令。调用方应在主循环中持续
 * 调用 ArmMotionState_Update()；状态机每 500 ms 读取全部关节。只要任一
 * 关节没有进入 ARM_MOTION_STATE_PWM_TOLERANCE 误差范围，或查询失败，
 * 就重新发送当前行的组位置命令；四轴全部到位后才下发下一行。
 *
 * @param state 非空的状态机实例。
 * @param sequence 非空且 frameCount 大于 0 的二维动作表描述。
 * @param now_ms 当前毫秒时间戳。
 * @return 首列命令均成功提交时返回 true，否则返回 false 且状态为 Failed。
 */
bool ArmMotionState_Start(ArmMotionState *state,
						  const ArmMotionState_Sequence *sequence,
						  uint32_t now_ms);

/**
 * @brief 推进一次状态机；位置查询时最多会阻塞四次单轴查询超时时间。
 * @return 当前运行、完成或失败状态。
 */
ArmMotionState_Status ArmMotionState_Update(ArmMotionState *state,
											uint32_t now_ms);

/** 将状态机复位为空闲状态。 */
void ArmMotionState_Reset(ArmMotionState *state);

#endif /* ARM_ARM_MOTION_STATE_H */
