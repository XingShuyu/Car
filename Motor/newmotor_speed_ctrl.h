#ifndef NEWMOTOR_SPEED_CTRL_H
#define NEWMOTOR_SPEED_CTRL_H

#include <stdint.h>

#include "newmotor_driver.h"

#ifndef NEWMOTOR_BALANCE_WHEEL_DIFF_TO_TURN_TICKS
#define NEWMOTOR_BALANCE_WHEEL_DIFF_TO_TURN_TICKS (0.20f)
#endif

#ifndef NEWMOTOR_BALANCE_SPIN_WHEEL_DIFF_TO_TURN_TICKS
#define NEWMOTOR_BALANCE_SPIN_WHEEL_DIFF_TO_TURN_TICKS (0.70f)
#endif

#ifndef NEWMOTOR_BALANCE_SPIN_FORWARD_EPS_MMPS
#define NEWMOTOR_BALANCE_SPIN_FORWARD_EPS_MMPS (20.0f)
#endif

#ifndef NEWMOTOR_BALANCE_SPIN_MIN_TURN_TICKS
#define NEWMOTOR_BALANCE_SPIN_MIN_TURN_TICKS (90.0f)
#endif

#ifndef NEWMOTOR_BALANCE_FORWARD_SIGN
#define NEWMOTOR_BALANCE_FORWARD_SIGN (-1.0f)
#endif

#ifndef NEWMOTOR_BALANCE_STRAIGHT_TURN_BIAS_TICKS
#define NEWMOTOR_BALANCE_STRAIGHT_TURN_BIAS_TICKS (30.0f)
#endif

#ifndef NEWMOTOR_BALANCE_TURN_PWM_LIMIT_TICKS
#define NEWMOTOR_BALANCE_TURN_PWM_LIMIT_TICKS (260.0f)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 单个电机的增量式 PID 状态。
 */
typedef struct NewMotor_IncPid {
    float kp;
    float ki;
    float kd;

    float sample_period_s;

    float target_mmps;
    float out_ticks;

    float err;
    float err_last;
    float err_next;

    float out_min_ticks;
    float out_max_ticks;
} NewMotor_IncPid;

/**
 * @brief 双轮差速底盘的速度闭环控制器。
 */
typedef struct NewMotor_SpeedCtrl {
    float sample_period_s;

    float target_left_mmps;
    float target_right_mmps;
    float target_forward_mmps;
    float target_turn_ticks;

    float measured_left_mmps;
    float measured_right_mmps;

    int16_t pwm_left_ticks;
    int16_t pwm_right_ticks;

    NewMotor_IncPid pid_left;
    NewMotor_IncPid pid_right;

    uint8_t enabled;
} NewMotor_SpeedCtrl;

/**
 * @brief 初始化速度闭环控制器。
 *
 * @param ctrl 控制器对象指针，不能为空。
 * @param sample_period_s 控制周期（单位：s），例如20ms填0.02f。
 */
void NewMotorSpeedCtrl_Init(NewMotor_SpeedCtrl *ctrl, float sample_period_s);

/**
 * @brief 设置左右电机 PID 参数（同一组参数）。
 *
 * @param ctrl 控制器对象指针，不能为空。
 * @param kp 比例系数。
 * @param ki 积分系数（增量式里对应误差累加项权重）。
 * @param kd 微分系数。
 */
void NewMotorSpeedCtrl_SetPid(NewMotor_SpeedCtrl *ctrl, float kp, float ki, float kd);

/**
 * @brief 设置左右电机 PID 输出限幅（单位：ticks）。
 *
 * @param ctrl 控制器对象指针，不能为空。
 * @param out_min_ticks 最小输出（通常为负值）。
 * @param out_max_ticks 最大输出（通常为正值）。
 */
void NewMotorSpeedCtrl_SetOutputLimit(NewMotor_SpeedCtrl *ctrl, float out_min_ticks, float out_max_ticks);

/**
 * @brief 设置左右轮目标线速度（单位：mm/s）。
 *
 * @param ctrl 控制器对象指针，不能为空。
 * @param left_mmps 左轮目标速度（mm/s，正前负后）。
 * @param right_mmps 右轮目标速度（mm/s，正前负后）。
 */
void NewMotorSpeedCtrl_SetTargetWheelMmps(NewMotor_SpeedCtrl *ctrl, float left_mmps, float right_mmps);

/**
 * @brief 以车体速度设置目标（差速底盘）。
 *
 * @param ctrl 控制器对象指针，不能为空。
 * @param vx_mmps 车体前向速度（mm/s）。
 * @param wz_radps 车体偏航角速度（rad/s，逆时针为正）。
 */
void NewMotorSpeedCtrl_SetTargetRobot(NewMotor_SpeedCtrl *ctrl, float vx_mmps, float wz_radps);

/**
 * @brief 根据编码器增量更新一次闭环并输出 PWM。
 *
 * @param ctrl 控制器对象指针，不能为空。
 * @param left_delta_counts 左轮本周期编码器增量计数（count）。
 * @param right_delta_counts 右轮本周期编码器增量计数（count）。
 */
void NewMotorSpeedCtrl_UpdateByEncoderDelta(NewMotor_SpeedCtrl *ctrl, int32_t left_delta_counts, int32_t right_delta_counts);

/**
 * @brief 清空 PID 内部状态并停车。
 *
 * @param ctrl 控制器对象指针，不能为空。
 * @param stop_mode 停止模式：滑行或刹车。
 */
void NewMotorSpeedCtrl_ResetAndStop(NewMotor_SpeedCtrl *ctrl, NewMotor_StopMode stop_mode);

/**
 * @brief 获取当前估计的左右轮速度（单位：mm/s）。
 *
 * @param ctrl 控制器对象指针，不能为空。
 * @param left_mmps 输出左轮速度指针，可为NULL。
 * @param right_mmps 输出右轮速度指针，可为NULL。
 */
void NewMotorSpeedCtrl_GetMeasuredWheelMmps(const NewMotor_SpeedCtrl *ctrl, float *left_mmps, float *right_mmps);

/**
 * @brief 获取当前输出 PWM（单位：ticks）。
 *
 * @param ctrl 控制器对象指针，不能为空。
 * @param left_ticks 输出左轮PWM指针，可为NULL。
 * @param right_ticks 输出右轮PWM指针，可为NULL。
 */
void NewMotorSpeedCtrl_GetOutputPwmTicks(const NewMotor_SpeedCtrl *ctrl, int16_t *left_ticks, int16_t *right_ticks);

#ifdef __cplusplus
}
#endif

#endif
