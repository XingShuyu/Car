#ifndef ARM_ARM_IK_MOTION_H
#define ARM_ARM_IK_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "Arm/arm_motion_state.h"

/**
 * @brief 笛卡尔自动朝下控制的标定配置。
 *
 * axis[0] 对应底座 yaw，axis[1..3] 对应肩、肘、腕俯仰。
 * axisDirection 为模型正方向到协议 PWM 正方向的映射，只允许 +1 或 -1。
 */
typedef struct ArmIkMotion_Calibration {
	float baseHeightMm;
	float link1Mm;
	float link2Mm;
	float link3Mm;
	float jointMinDeg[3];
	float jointMaxDeg[3];
	float axisZeroDeg[4];
	int8_t axisDirection[4];
	uint16_t axisMinPwm[4];
	uint16_t axisMaxPwm[4];
	uint16_t fixedPwm[2]; /* ID 4 腕旋转、ID 5 夹爪。 */
	uint16_t positionReadTimeoutMs;
} ArmIkMotion_Calibration;

/**
 * @brief 设置下一次笛卡尔动作使用的几何和舵机标定参数。
 *
 * 仅能在当前 IK 动作空闲、完成或失败后设置；运行中调用返回 false。
 */
bool ArmIkMotion_SetCalibration(const ArmIkMotion_Calibration *calibration);

/**
 * @brief 读取当前标定副本，便于调用方仅修改方向、零位或限位后再写回。
 *
 * @param calibration 非空时写入当前使用的标定；NULL 时不执行任何操作。
 */
void ArmIkMotion_GetCalibration(ArmIkMotion_Calibration *calibration);

/**
 * @brief 启动一次自动优先钩爪朝下的笛卡尔动作。
 *
 * yaw=0 指向车头正前方，逆时针（向车左）为正；x 是底座偏航后的水平
 * 径向距离（mm），y 是相对底座旋转中心向上的高度（mm）。函数会在所有
 * 合法解析候选中优先选择钩爪方向最接近竖直向下（180 度）的姿态；方向
 * 误差相同时，再选择移动量最小的肘部解。随后将六轴 PWM 目标交给非阻塞
 * 状态机执行。
 *
 * @return 成功启动返回 true；坐标不可达、标定/位置读取失败或已有动作运行
 *         时返回 false，且不会下发新的位置命令。
 */
bool ArmIkMotion_Start(float yaw_deg, float x_mm, float y_mm);

/**
 * @brief 返回最近一次 Start 失败是否由目标不可达或角度/PWM 限位导致。
 *
 * 位置读取、UART 发送、标定或忙状态等失败时返回 false。
 */
bool ArmIkMotion_LastStartWasReachabilityFailure(void);

/** @brief 推进当前动作并返回其到位状态。 */
ArmMotionState_Status ArmIkMotion_Update(void);

/** @brief 丢弃当前 IK 动作状态，不再继续重发其目标位置。 */
void ArmIkMotion_Reset(void);

#endif /* ARM_ARM_IK_MOTION_H */
