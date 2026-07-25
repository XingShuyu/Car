#ifndef ARM_ARM_TARGET_TEST_H
#define ARM_ARM_TARGET_TEST_H

#include <stdbool.h>

/** 简单硬编码目标测试状态。 */
typedef enum ArmTargetTest_Status {
	ArmTargetTestIdle = 0,
	ArmTargetTestMoving,
	ArmTargetTestCompleted,
	ArmTargetTestCannotReach,
	ArmTargetTestFailed,
} ArmTargetTest_Status;

/** 启动一次自动优先钩爪朝下的 IK 目标，仅在空闲、完成或失败状态允许调用。 */
bool ArmTargetTest_SubmitTarget(float yaw_deg, float x_mm, float y_mm);

/** 推进一次机械臂到位状态。 */
ArmTargetTest_Status ArmTargetTest_Update(void);

/** 复位测试状态机；不向机械臂发送停止命令。 */
void ArmTargetTest_Reset(void);

/**
 * @brief 菜单中的硬编码目标测试，进入时提交一个固定 IK 目标，B1 退出。
 */
void ArmTargetTest_Run(void);

#endif /* ARM_ARM_TARGET_TEST_H */
