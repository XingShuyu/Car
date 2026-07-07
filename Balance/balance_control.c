#include "balance_control.h"

#include <math.h>
#include <stddef.h>

#include "Motor/newmotor_driver.h"

#define STANDUP_SAFE_ANGLE_DEG 15.0f
#define STANDUP_DT_MAX_S 0.05f
#define STANDUP_PWM_LIMIT_TICKS 1800
#define STANDUP_KP_TICKS_PER_DEG 19.6f
#define STANDUP_KD_TICKS_PER_DEGPS 2.0f
#define STANDUP_KG_TICKS_PER_DEGPS 7.0f
#define STANDUP_KGD_TICKS_PER_DEGPS 1.5f
#define STANDUP_D_FILTER_ALPHA 0.0f
#define STANDUP_LOG_INTERVAL_MS 50U
#define STANDUP_DISPLAY_INTERVAL_MS 100U

#define BALANCE_DEFAULT_CENTER_ANGLE_DEG 3.0f

/* 倾角超过该值认为车体已经倒下，立即刹车并复位平衡目标。 */
#define BALANCE_TILT_STOP_ANGLE_DEG 30.0f
#define BALANCE_DT_MAX_S STANDUP_DT_MAX_S

/* 速度外环：把速度误差换算成一个很小的目标角偏移。 */
#define BALANCE_SPEED_OUTER_KP 0.003f
#define BALANCE_SPEED_ANGLE_LIMIT_DEG 5.0f

/*
 * 启动中心角标定：
 * - 只在 CALIBRATING 状态学习中心角。
 * - 目标静止且车体稳定时，把速度外环修正角慢慢并入中心角。
 * - 标定完成后进入 RUNNING，运行期不再持续修改中心角。
 */
#define BALANCE_CENTER_LIMIT_DEG 6.0f
#define BALANCE_CALIBRATE_TIME_S 0.80f
#define BALANCE_CALIBRATE_MIN_SAMPLES 20U
#define BALANCE_CALIBRATE_CENTER_LEARN_ALPHA 0.02f
#define BALANCE_CALIBRATE_GYRO_LIMIT_DPS 8.0f
#define BALANCE_CALIBRATE_ROLL_ERROR_LIMIT_DEG 4.0f
#define BALANCE_CALIBRATE_SPEED_LIMIT_MMPS 20.0f
#define BALANCE_CALIBRATE_TARGET_SPEED_LIMIT_MMPS 20.0f
#define BALANCE_CALIBRATE_TURN_LIMIT_TICKS 20.0f
#define BALANCE_SPEED_CMD_SLEW_MMPS2 220.0f
#define BALANCE_TURN_CMD_SLEW_TICKS_PER_S 420.0f
#define BALANCE_TURN_PWM_LIMIT_TICKS 260.0f
#define BALANCE_SPEED_FILTER_ALPHA 0.25f
#define BALANCE_TURN_DISABLE_ROLL_ERROR_DEG 8.0f
#define BALANCE_LOG_INTERVAL_MS 100U

/* 内环增益：横滚角、横滚角速度和速度误差最终都换算成 PWM 计数值。 */
#define BALANCE_INNER_ROLL_SCALE_TICKS_PER_DEG (2000.0f / 20.0f)
#define BALANCE_INNER_GYRO_GAIN_TICKS_PER_DEGPS 6.5f
#define BALANCE_INNER_SPEED_GAIN_TICKS_PER_MMPS 3.0f

static BalanceControl_State balance_state;
static uint16_t calibrate_sample_count;
static float calibrate_stable_time_s;

static float clampf_local(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float slew_to_target(float current, float target, float rate, float dt)
{
    float max_step = rate * dt;
    float delta = target - current;

    if (max_step <= 0.0f) {
        return target;
    }
    if (delta > max_step) {
        return current + max_step;
    }
    if (delta < -max_step) {
        return current - max_step;
    }
    return target;
}

static void reset_calibration_accumulator(void)
{
    calibrate_sample_count = 0U;
    calibrate_stable_time_s = 0.0f;
    balance_state.calibrate_progress = 0.0f;
}

static bool is_body_upright_for_restart(const IMU_Data_t *imu)
{
    return fabsf(imu->roll) < STANDUP_SAFE_ANGLE_DEG &&
           fabsf(imu->gx) < BALANCE_CALIBRATE_GYRO_LIMIT_DPS;
}

static bool is_calibration_target_idle(const NewMotor_SpeedCtrl *motor_ctrl)
{
    return fabsf(motor_ctrl->target_forward_mmps) <
               BALANCE_CALIBRATE_TARGET_SPEED_LIMIT_MMPS &&
           fabsf(balance_state.speed_cmd_mmps) <
               BALANCE_CALIBRATE_TARGET_SPEED_LIMIT_MMPS &&
           fabsf(motor_ctrl->target_turn_ticks) <
               BALANCE_CALIBRATE_TURN_LIMIT_TICKS &&
           fabsf(balance_state.turn_cmd_ticks) <
               BALANCE_CALIBRATE_TURN_LIMIT_TICKS;
}

static bool is_calibration_body_stable(const IMU_Data_t *imu)
{
    return fabsf(imu->gx) < BALANCE_CALIBRATE_GYRO_LIMIT_DPS &&
           fabsf(imu->roll - balance_state.target_angle_deg) <
               BALANCE_CALIBRATE_ROLL_ERROR_LIMIT_DEG;
}

static bool is_calibration_wheel_stable(void)
{
    return fabsf(balance_state.avg_speed_filtered_mmps) <
           BALANCE_CALIBRATE_SPEED_LIMIT_MMPS;
}

static void update_start_calibration(const NewMotor_SpeedCtrl *motor_ctrl,
                                     const IMU_Data_t *imu,
                                     float dt_s)
{
    float outer_period_s =
        (float)BALANCE_CONTROL_OUTER_PERIOD_US / 1000000.0f;
    float learn_alpha =
        BALANCE_CALIBRATE_CENTER_LEARN_ALPHA * dt_s / outer_period_s;

    /*
     * 启动标定分两步：
     * 1. 只要目标静止且车体姿态稳定，就把速度外环给出的修正角慢慢并入
     *    center_angle_deg。这样小车还在向前/向后漂时，也能继续修正中心角。
     * 2. 只有轮速也接近 0 并持续一段时间，才认为中心角已经学好。
     *
     * 原来只在轮速已经接近 0 时才累计，容易导致 speed_angle_deg 已经变小，
     * 最后学到的中心角仍然接近默认值。
     */
    if (!is_calibration_target_idle(motor_ctrl) ||
        !is_calibration_body_stable(imu)) {
        reset_calibration_accumulator();
        return;
    }

    learn_alpha = clampf_local(learn_alpha, 0.0f, 1.0f);
    balance_state.center_angle_deg = clampf_local(
        balance_state.center_angle_deg +
            learn_alpha * balance_state.speed_angle_deg,
        -BALANCE_CENTER_LIMIT_DEG,
        BALANCE_CENTER_LIMIT_DEG);

    if (!is_calibration_wheel_stable()) {
        calibrate_sample_count = 0U;
        calibrate_stable_time_s = 0.0f;
        balance_state.calibrate_progress = 0.0f;
        return;
    }

    if (calibrate_sample_count < UINT16_MAX) {
        calibrate_sample_count++;
    }

    calibrate_stable_time_s += dt_s;
    balance_state.calibrate_progress =
        clampf_local(calibrate_stable_time_s / BALANCE_CALIBRATE_TIME_S,
                     0.0f,
                     1.0f);

    if (calibrate_stable_time_s >= BALANCE_CALIBRATE_TIME_S &&
        calibrate_sample_count >= BALANCE_CALIBRATE_MIN_SAMPLES) {
        balance_state.speed_angle_deg = 0.0f;
        balance_state.target_angle_deg = balance_state.center_angle_deg;
        balance_state.mode = BALANCE_CONTROL_MODE_RUNNING;
        balance_state.calibrate_progress = 1.0f;
    }
}

static void clear_output(BalanceControl_Output *output)
{
    if (output == NULL) {
        return;
    }

    output->left_speed_mmps = 0.0f;
    output->right_speed_mmps = 0.0f;
    output->avg_speed_mmps = 0.0f;
    output->left_pwm_ticks = 0;
    output->right_pwm_ticks = 0;
    output->stopped_for_tilt = false;
}

void BalanceControl_Init(void)
{
    BalanceControl_ResetTarget(true);
}

void BalanceControl_ResetTarget(bool reset_center)
{
    if (reset_center) {
        /* 完整复位：回到默认中心角，并重新进入启动标定。 */
        balance_state.center_angle_deg = BALANCE_DEFAULT_CENTER_ANGLE_DEG;
        balance_state.mode = BALANCE_CONTROL_MODE_CALIBRATING;
        reset_calibration_accumulator();
    }

    /* reset_center 为 false 时保留中心角，只清空速度/转向目标。 */
    balance_state.speed_angle_deg = 0.0f;
    balance_state.target_angle_deg = balance_state.center_angle_deg;
    balance_state.speed_cmd_mmps = 0.0f;
    balance_state.turn_cmd_ticks = 0.0f;
    balance_state.avg_speed_filtered_mmps = 0.0f;
}

void BalanceControl_UpdateTarget(const NewMotor_SpeedCtrl *motor_ctrl,
                                 const IMU_Data_t *imu,
                                 float dt_s)
{
    float speed_error;

    if (motor_ctrl == NULL || imu == NULL) {
        return;
    }

    if (balance_state.mode == BALANCE_CONTROL_MODE_FAULT) {
        balance_state.speed_angle_deg = 0.0f;
        balance_state.target_angle_deg = balance_state.center_angle_deg;
        balance_state.speed_cmd_mmps = 0.0f;
        balance_state.turn_cmd_ticks = 0.0f;

        /*
         * 故障态下不恢复电机输出。只有重新扶正并保持基本稳定，才回到
         * 启动标定态，重新学习一次中心角。
         */
        if (is_body_upright_for_restart(imu)) {
            balance_state.mode = BALANCE_CONTROL_MODE_CALIBRATING;
            reset_calibration_accumulator();
        }
        return;
    }

    /*
     * 阶段状态机会直接修改 motor_ctrl 的目标速度/转向。这里先做斜率限制，
     * 避免目标突变导致车体突然前倾、后仰或急转。
     */
    balance_state.speed_cmd_mmps = slew_to_target(
        balance_state.speed_cmd_mmps,
        motor_ctrl->target_forward_mmps,
        BALANCE_SPEED_CMD_SLEW_MMPS2,
        dt_s);
    balance_state.turn_cmd_ticks = slew_to_target(
        balance_state.turn_cmd_ticks,
        clampf_local(motor_ctrl->target_turn_ticks,
                     -BALANCE_TURN_PWM_LIMIT_TICKS,
                     BALANCE_TURN_PWM_LIMIT_TICKS),
        BALANCE_TURN_CMD_SLEW_TICKS_PER_S,
        dt_s);

    /*
     * 速度外环：
     * 实际速度 - 目标速度 -> 目标倾角修正。限幅是为了让修正量保持很小，
     * 避免速度环抢过直立内环的控制权。
     */
    speed_error =
        balance_state.avg_speed_filtered_mmps - balance_state.speed_cmd_mmps;
    balance_state.speed_angle_deg = clampf_local(
        -BALANCE_SPEED_OUTER_KP * speed_error,
        -BALANCE_SPEED_ANGLE_LIMIT_DEG,
        BALANCE_SPEED_ANGLE_LIMIT_DEG);

    /* 先写入本周期临时目标角，供标定稳定性判断使用。 */
    balance_state.target_angle_deg =
        balance_state.center_angle_deg + balance_state.speed_angle_deg;

    if (balance_state.mode == BALANCE_CONTROL_MODE_CALIBRATING) {
        update_start_calibration(motor_ctrl, imu, dt_s);
    }

    /* 5ms 内环实际跟踪的横滚目标角。标定阶段会使用刚更新后的中心角。 */
    balance_state.target_angle_deg =
        balance_state.center_angle_deg + balance_state.speed_angle_deg;
}

bool BalanceControl_UpdateFast(NewMotor_SpeedCtrl *motor_ctrl,
                               const IMU_Data_t *imu,
                               int32_t left_delta_counts,
                               int32_t right_delta_counts,
                               float dt_s,
                               BalanceControl_Output *output)
{
    int left_speed_mmps;
    int right_speed_mmps;
    float avg_speed_mmps;
    float speed_error;
    float roll_error;
    float turn_ticks;
    int out_ticks;
    int16_t left_pwm_ticks;
    int16_t right_pwm_ticks;

    clear_output(output);

    if (motor_ctrl == NULL || imu == NULL) {
        return false;
    }

    if (balance_state.mode == BALANCE_CONTROL_MODE_FAULT) {
        NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
        if (output != NULL) {
            output->stopped_for_tilt = true;
        }
        return false;
    }

    /* 同步采样周期，保证其它读取 motor_ctrl 的代码看到一致的周期。 */
    motor_ctrl->sample_period_s = dt_s;

    /* 将本周期编码器增量换算成左右轮速度。 */
    left_speed_mmps =
        (int)(NewMotor_EncoderDeltaToDistanceMm(left_delta_counts) / dt_s);
    right_speed_mmps =
        (int)(NewMotor_EncoderDeltaToDistanceMm(right_delta_counts) / dt_s);
    avg_speed_mmps =
        0.5f * ((float)left_speed_mmps + (float)right_speed_mmps);

    balance_state.avg_speed_filtered_mmps +=
        BALANCE_SPEED_FILTER_ALPHA *
        (avg_speed_mmps - balance_state.avg_speed_filtered_mmps);

    if (output != NULL) {
        output->left_speed_mmps = (float)left_speed_mmps;
        output->right_speed_mmps = (float)right_speed_mmps;
        output->avg_speed_mmps = avg_speed_mmps;
    }

    roll_error = imu->roll - balance_state.target_angle_deg;
    speed_error =
        balance_state.avg_speed_filtered_mmps - balance_state.speed_cmd_mmps;

    /* 倾角过大时优先刹车复位，避免倒下后继续输出 PWM。 */
    if (imu->roll > BALANCE_TILT_STOP_ANGLE_DEG ||
        imu->roll < -BALANCE_TILT_STOP_ANGLE_DEG) {
        BalanceControl_ResetTarget(true);
        balance_state.mode = BALANCE_CONTROL_MODE_FAULT;
        NewMotorSpeedCtrl_SetTargetWheelMmps(motor_ctrl, 0.0f, 0.0f);
        NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
        if (output != NULL) {
            output->stopped_for_tilt = true;
        }
        return false;
    }

    /*
     * 直立内环：
     * - 横滚角误差项把车体拉回目标角 target_angle_deg。
     * - gx 项抑制角速度，相当于阻尼。
     * - speed_error 项补偿单纯角度环看不到的慢速漂移。
     */
    out_ticks =
        (int)(roll_error * BALANCE_INNER_ROLL_SCALE_TICKS_PER_DEG -
              BALANCE_INNER_GYRO_GAIN_TICKS_PER_DEGPS * imu->gx +
              BALANCE_INNER_SPEED_GAIN_TICKS_PER_MMPS * speed_error);

    /* 横滚角误差较大时禁用转向补偿，先保证直立稳定。 */
    turn_ticks = balance_state.turn_cmd_ticks;
    if (fabsf(roll_error) > BALANCE_TURN_DISABLE_ROLL_ERROR_DEG) {
        turn_ticks = 0.0f;
    }

    left_pwm_ticks = (int16_t)((float)out_ticks - turn_ticks);
    right_pwm_ticks = (int16_t)((float)out_ticks + turn_ticks);
    NewMotor_SetWheelPwmTicks(left_pwm_ticks, right_pwm_ticks);

    if (output != NULL) {
        output->left_pwm_ticks = left_pwm_ticks;
        output->right_pwm_ticks = right_pwm_ticks;
    }

    return true;
}

const BalanceControl_State *BalanceControl_GetState(void)
{
    return &balance_state;
}

BalanceControl_Mode BalanceControl_GetMode(void)
{
    return balance_state.mode;
}

bool BalanceControl_IsRunning(void)
{
    return balance_state.mode == BALANCE_CONTROL_MODE_RUNNING;
}

float BalanceControl_GetFilteredSpeedMmps(void)
{
    return balance_state.avg_speed_filtered_mmps;
}
