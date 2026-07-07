#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "IMU/imu_data.h"
#include "Motor/newmotor_speed_ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BALANCE_CONTROL_FAST_PERIOD_US 5000U
#define BALANCE_CONTROL_OUTER_PERIOD_US 20000U

typedef enum BalanceControl_Mode {
    BALANCE_CONTROL_MODE_CALIBRATING = 0,
    BALANCE_CONTROL_MODE_RUNNING,
    BALANCE_CONTROL_MODE_FAULT
} BalanceControl_Mode;

typedef struct BalanceControl_State {
    /* 当前平衡状态：启动标定、正常运行或故障停机。 */
    BalanceControl_Mode mode;
    /* 启动标定进度，0.0 表示未开始，1.0 表示标定完成。 */
    float calibrate_progress;
    /* 启动标定得到的直立中心角，用来吸收传感器零点或重心偏差。 */
    float center_angle_deg;
    /* 速度外环给出的目标角修正量，正负方向跟随横滚角定义。 */
    float speed_angle_deg;
    /* 内环最终横滚目标角：center_angle_deg + speed_angle_deg。 */
    float target_angle_deg;
    /* 从 NewMotor_SpeedCtrl 目标值平滑得到的前进速度命令。 */
    float speed_cmd_mmps;
    /* 从 NewMotor_SpeedCtrl 目标值平滑得到的转向 PWM 补偿。 */
    float turn_cmd_ticks;
    /* 编码器平均速度的一阶滤波值，供速度外环和启动标定使用。 */
    float avg_speed_filtered_mmps;
} BalanceControl_State;

typedef struct BalanceControl_Output {
    float left_speed_mmps;
    float right_speed_mmps;
    float avg_speed_mmps;
    int16_t left_pwm_ticks;
    int16_t right_pwm_ticks;
    bool stopped_for_tilt;
} BalanceControl_Output;

void BalanceControl_Init(void);
void BalanceControl_ResetTarget(bool reset_center);
/* 20ms 外环：更新速度/转向目标，并在启动阶段标定直立中心角。 */
void BalanceControl_UpdateTarget(const NewMotor_SpeedCtrl *motor_ctrl,
                                 const IMU_Data_t *imu,
                                 float dt_s);
/* 5ms 内环：根据编码器和 IMU 计算平衡 PWM 并输出到电机。 */
bool BalanceControl_UpdateFast(NewMotor_SpeedCtrl *motor_ctrl,
                               const IMU_Data_t *imu,
                               int32_t left_delta_counts,
                               int32_t right_delta_counts,
                               float dt_s,
                               BalanceControl_Output *output);
const BalanceControl_State *BalanceControl_GetState(void);
BalanceControl_Mode BalanceControl_GetMode(void);
bool BalanceControl_IsRunning(void);
float BalanceControl_GetFilteredSpeedMmps(void);

#ifdef __cplusplus
}
#endif

#endif
