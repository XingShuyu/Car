#ifndef ARM_JIBOT_SERVO_H
#define ARM_JIBOT_SERVO_H

#include <stdbool.h>
#include <stdint.h>

/* Jibot 总线舵机 ID：从机械臂底座到夹爪依次为 000~005。 */
enum {
	JIBOT_SERVO_ID_BASE = 0U,
	JIBOT_SERVO_ID_SHOULDER = 1U,
	JIBOT_SERVO_ID_ELBOW = 2U,
	JIBOT_SERVO_ID_WRIST_PITCH = 3U,
	JIBOT_SERVO_ID_WRIST_ORIENTATION = 4U,
	JIBOT_SERVO_ID_GRIPPER = 5U,
	JIBOT_SERVO_COUNT = 6U,
};

/* Jibot 270 度位置模式的协议参数范围。 */
#define JIBOT_SERVO_MIN_ANGLE_DEG (-135.0F)
#define JIBOT_SERVO_MAX_ANGLE_DEG (135.0F)
#define JIBOT_SERVO_MIN_PWM       (500U)
#define JIBOT_SERVO_MAX_PWM       (2500U)
#define JIBOT_SERVO_MAX_TIME_MS   (9999U)

/** 任意两条完整 Jibot 指令之间的最小发送间隔。 */
#define JIBOT_SERVO_COMMAND_INTERVAL_MS (1U)

/**
 * @brief 按协议原始 PWM 值控制一个 Jibot 总线舵机。
 *
 * @param id 舵机 ID，允许 000~005。
 * @param position_pwm 目标 PWM，允许 500~2500。
 * @param time_ms 到达目标的时间，允许 0~9999 ms。
 * @return 参数合法且命令已提交给 UART0 发送时返回 true；否则返回 false。
 */
bool JibotServo_SetPwm(uint8_t id, uint16_t position_pwm,
					   uint16_t time_ms);

/**
 * @brief 将六轴位置命令打包为一条 Jibot 组命令并发送。
 *
 * 发送格式为 `{G0000#000PppppTtttt!#001PppppTtttt!...#005PppppTtttt!}`。
 * 组命令使六个舵机控制指令连续且完整地到达控制器，适用于一帧六轴同步动作。
 *
 * @param position_pwm 六个舵机 ID 0~5 的目标 PWM，均须在 500~2500。
 * @param time_ms 全部舵机共用的到达时间，允许 0~9999 ms。
 * @return 参数合法且整条组命令已提交给 UART0 发送时返回 true；否则返回 false。
 */
bool JibotServo_SetPwmBatch(const uint16_t position_pwm[JIBOT_SERVO_COUNT],
							uint16_t time_ms);

/**
 * @brief 按中心相对角度控制一个 Jibot 总线舵机。
 *
 * 0 度对应协议 PWM=1500；-135 度对应 PWM=500；+135 度对应 PWM=2500。
 * 正角度表示 PWM 数值增大。各关节实际机械正方向和安全限位需要实机标定。
 *
 * @param id 舵机 ID，允许 000~005。
 * @param angle_deg 目标相对角度，允许 -135.0~+135.0 度。
 * @param time_ms 到达目标的时间，允许 0~9999 ms。
 * @return 参数合法且命令已提交给 UART0 发送时返回 true；否则返回 false。
 */
bool JibotServo_SetAngle(uint8_t id, float angle_deg, uint16_t time_ms);

/**
 * @brief 释放一个 Jibot 舵机的保持扭力。
 *
 * 对应协议命令 #dddPULK!。释放后关节可被外力带动，调用方应先确认
 * 机械臂已得到支撑，避免因重力下落、夹伤或碰撞。
 *
 * @param id 舵机 ID，允许 000~005。
 * @return 参数合法且命令已提交给 UART0 发送时返回 true；否则返回 false。
 */
bool JibotServo_ReleaseTorque(uint8_t id);

/**
 * @brief 恢复一个 Jibot 舵机的保持扭力。
 *
 * 对应协议命令 #dddPULR!。恢复扭力时舵机会保持其当前实际位置，
 * 不会执行新的目标位置运动。
 *
 * @param id 舵机 ID，允许 000~005。
 * @return 参数合法且命令已提交给 UART2 发送时返回 true；否则返回 false。
 */
bool JibotServo_RestoreTorque(uint8_t id);

/**
 * @brief 读取一个 Jibot 舵机的当前位置 PWM 值。
 *
 * 发送 #dddPRAD!，并轮询 UART0 等待 #dddPpppp! 应答。返回值为协议中的
 * 原始 PWM 位置（通常 500~2500）；需要中心相对角时可按
 * (pwm - 1500) * 270 / 2000 换算。
 *
 * 本函数为阻塞式轮询：调用期间不会执行主循环调度，建议超时设置为
 * 20~100 ms，且只能在 TimeBase_Init() 完成后调用。超时为 0 时只检查
 * 查询发送后已进入 UART FIFO 的数据，不额外等待。
 *
 * @param id 舵机 ID，允许 000~005。
 * @param position_pwm 成功时写入四位 PWM 位置值；不得为 NULL。
 * @param timeout_ms 等待应答的最长时间，单位 ms。
 * @return 收到 ID 匹配且格式正确的应答时返回 true；超时或参数无效返回 false。
 */
bool JibotServo_ReadPosition(uint8_t id, uint16_t *position_pwm,
					 uint16_t timeout_ms);

#endif
