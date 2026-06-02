#include "newmotor_speed_ctrl.h"

#include <math.h>
#include <stddef.h>

#define NM_STOP_TARGET_EPSILON_MMPS (0.5f)

static float nm_clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static void nm_pid_reset(NewMotor_IncPid *pid)
{
    pid->out_ticks = 0.0f;
    pid->err = 0.0f;
    pid->err_last = 0.0f;
    pid->err_next = 0.0f;
}

static float nm_pid_incre_step(NewMotor_IncPid *pid, float measured_mmps)
{
    pid->err = pid->target_mmps - measured_mmps;
    pid->out_ticks += pid->kp * (pid->err - pid->err_next) +
                      pid->ki * pid->sample_period_s * pid->err +
                      pid->kd / pid->sample_period_s * (pid->err - 2.0f * pid->err_next + pid->err_last);

    pid->err_last = pid->err_next;
    pid->err_next = pid->err;
    pid->out_ticks = nm_clampf(pid->out_ticks, pid->out_min_ticks, pid->out_max_ticks);
    return pid->out_ticks;
}

void NewMotorSpeedCtrl_Init(NewMotor_SpeedCtrl *ctrl, float sample_period_s)
{
    if (ctrl == NULL) {
        return;
    }

    ctrl->sample_period_s = (sample_period_s > 0.0f) ? sample_period_s : 0.02f;

    ctrl->target_left_mmps = 0.0f;
    ctrl->target_right_mmps = 0.0f;
    ctrl->measured_left_mmps = 0.0f;
    ctrl->measured_right_mmps = 0.0f;
    ctrl->pwm_left_ticks = 0;
    ctrl->pwm_right_ticks = 0;
    ctrl->enabled = 1;

    ctrl->pid_left.kp = 1.5f;
    ctrl->pid_left.ki = 0.0f;
    ctrl->pid_left.kd = 0.01f;
    ctrl->pid_left.target_mmps = 0.0f;
    ctrl->pid_left.out_min_ticks = -(float)(NEWMOTOR_PWM_MAX_TICKS - NEWMOTOR_PWM_DEADZONE_TICKS);
    ctrl->pid_left.out_max_ticks = (float)(NEWMOTOR_PWM_MAX_TICKS - NEWMOTOR_PWM_DEADZONE_TICKS);

    ctrl->pid_right = ctrl->pid_left;

    nm_pid_reset(&ctrl->pid_left);
    nm_pid_reset(&ctrl->pid_right);
}

void NewMotorSpeedCtrl_SetPid(NewMotor_SpeedCtrl *ctrl, float kp, float ki, float kd)
{
    if (ctrl == NULL) {
        return;
    }

    ctrl->pid_left.kp = kp;
    ctrl->pid_left.ki = ki;
    ctrl->pid_left.kd = kd;

    ctrl->pid_right.kp = kp;
    ctrl->pid_right.ki = ki;
    ctrl->pid_right.kd = kd;
}

void NewMotorSpeedCtrl_SetOutputLimit(NewMotor_SpeedCtrl *ctrl, float out_min_ticks, float out_max_ticks)
{
    if (ctrl == NULL) {
        return;
    }
    if (out_min_ticks > out_max_ticks) {
        return;
    }

    ctrl->pid_left.out_min_ticks = out_min_ticks;
    ctrl->pid_left.out_max_ticks = out_max_ticks;
    ctrl->pid_right.out_min_ticks = out_min_ticks;
    ctrl->pid_right.out_max_ticks = out_max_ticks;
}

void NewMotorSpeedCtrl_SetTargetWheelMmps(NewMotor_SpeedCtrl *ctrl, float left_mmps, float right_mmps)
{
    if (ctrl == NULL) {
        return;
    }

    ctrl->target_left_mmps = left_mmps;
    ctrl->target_right_mmps = right_mmps;

    ctrl->pid_left.target_mmps = left_mmps;
    ctrl->pid_right.target_mmps = right_mmps;
}

void NewMotorSpeedCtrl_SetTargetRobot(NewMotor_SpeedCtrl *ctrl, float vx_mmps, float wz_radps)
{
    float spin_term;

    if (ctrl == NULL) {
        return;
    }

    spin_term = 0.001f * NEWMOTOR_WHEEL_BASE_MM * wz_radps;
    NewMotorSpeedCtrl_SetTargetWheelMmps(ctrl, vx_mmps - spin_term, vx_mmps + spin_term);
}

void NewMotorSpeedCtrl_UpdateByEncoderDelta(NewMotor_SpeedCtrl *ctrl, int32_t left_delta_counts, int32_t right_delta_counts)
{
    float left_out;
    float right_out;

    if (ctrl == NULL) {
        return;
    }

    ctrl->measured_left_mmps = NewMotor_EncoderDeltaToWheelSpeedMmps(left_delta_counts, ctrl->sample_period_s);
    ctrl->pid_left.sample_period_s = ctrl->sample_period_s;
    ctrl->measured_right_mmps = NewMotor_EncoderDeltaToWheelSpeedMmps(right_delta_counts, ctrl->sample_period_s);
    ctrl->pid_right.sample_period_s = ctrl->sample_period_s;

    if (!ctrl->enabled) {
        ctrl->pwm_left_ticks = 0;
        ctrl->pwm_right_ticks = 0;
        NewMotor_SetWheelPwmTicksRaw(0, 0);
        return;
    }

    if (fabsf(ctrl->target_left_mmps) <= NM_STOP_TARGET_EPSILON_MMPS) {
        nm_pid_reset(&ctrl->pid_left);
        left_out = 0.0f;
    } else {
        left_out = nm_pid_incre_step(&ctrl->pid_left, ctrl->measured_left_mmps);
    }

    if (fabsf(ctrl->target_right_mmps) <= NM_STOP_TARGET_EPSILON_MMPS) {
        nm_pid_reset(&ctrl->pid_right);
        right_out = 0.0f;
    } else {
        right_out = nm_pid_incre_step(&ctrl->pid_right, ctrl->measured_right_mmps);
    }

    ctrl->pwm_left_ticks = (int16_t)left_out;
    ctrl->pwm_right_ticks = (int16_t)right_out;

    NewMotor_SetWheelPwmTicks(ctrl->pwm_left_ticks, ctrl->pwm_right_ticks);
}

void NewMotorSpeedCtrl_ResetAndStop(NewMotor_SpeedCtrl *ctrl, NewMotor_StopMode stop_mode)
{
    if (ctrl == NULL) {
        return;
    }

    NewMotorSpeedCtrl_SetTargetWheelMmps(ctrl, 0.0f, 0.0f);
    nm_pid_reset(&ctrl->pid_left);
    nm_pid_reset(&ctrl->pid_right);

    ctrl->pwm_left_ticks = 0;
    ctrl->pwm_right_ticks = 0;
    ctrl->enabled = 0;

    NewMotor_Stop(stop_mode);
}

void NewMotorSpeedCtrl_GetMeasuredWheelMmps(const NewMotor_SpeedCtrl *ctrl, float *left_mmps, float *right_mmps)
{
    if (ctrl == NULL) {
        return;
    }

    if (left_mmps != NULL) {
        *left_mmps = ctrl->measured_left_mmps;
    }
    if (right_mmps != NULL) {
        *right_mmps = ctrl->measured_right_mmps;
    }
}

void NewMotorSpeedCtrl_GetOutputPwmTicks(const NewMotor_SpeedCtrl *ctrl, int16_t *left_ticks, int16_t *right_ticks)
{
    if (ctrl == NULL) {
        return;
    }

    if (left_ticks != NULL) {
        *left_ticks = ctrl->pwm_left_ticks;
    }
    if (right_ticks != NULL) {
        *right_ticks = ctrl->pwm_right_ticks;
    }
}
