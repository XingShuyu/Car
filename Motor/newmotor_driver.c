#include "newmotor_driver.h"

#if defined(motor_PWM_INST) && defined(GPIO_motor_PWM_C0_IDX) && \
    defined(GPIO_motor_PWM_C1_IDX) && defined(GPIO_motor_PWM_C2_IDX) && defined(GPIO_motor_PWM_C3_IDX)
#define NEWMOTOR_BACKEND_TIMA4CH (1)
#elif defined(MotorLeft_INST) && defined(MotorRight_INST) && \
      defined(GPIO_MotorLeft_C0_IDX) && defined(GPIO_MotorLeft_C1_IDX) && \
      defined(GPIO_MotorRight_C0_IDX) && defined(GPIO_MotorRight_C1_IDX)
#define NEWMOTOR_BACKEND_DUAL_TIMERG (1)
#else
#error "No supported motor pin/timer mapping found in ti_msp_dl_config.h"
#endif

static uint16_t nm_abs_i16(int16_t v)
{
    return (v < 0) ? (uint16_t)(-v) : (uint16_t)v;
}

int16_t NewMotor_ClampSignedPwmTicks(int16_t ticks)
{
    if (ticks > NEWMOTOR_PWM_MAX_TICKS) {
        return NEWMOTOR_PWM_MAX_TICKS;
    }
    if (ticks < -NEWMOTOR_PWM_MAX_TICKS) {
        return -NEWMOTOR_PWM_MAX_TICKS;
    }
    return ticks;
}

int16_t NewMotor_ApplyDeadzoneTicks(int16_t ticks)
{
    int16_t clamped = NewMotor_ClampSignedPwmTicks(ticks);
    if (clamped > 0) {
        int32_t out = (int32_t)clamped + NEWMOTOR_PWM_DEADZONE_TICKS;
        if (out > NEWMOTOR_PWM_MAX_TICKS) {
            out = NEWMOTOR_PWM_MAX_TICKS;
        }
        return (int16_t)out;
    }

    if (clamped < 0) {
        int32_t out = (int32_t)clamped - NEWMOTOR_PWM_DEADZONE_TICKS;
        if (out < -NEWMOTOR_PWM_MAX_TICKS) {
            out = -NEWMOTOR_PWM_MAX_TICKS;
        }
        return (int16_t)out;
    }

    return 0;
}

static void nm_write_left(uint16_t pwm_ticks, bool reverse)
{
#if NEWMOTOR_BACKEND_TIMA4CH
    if (reverse) {
        DL_TimerA_setCaptureCompareValue(motor_PWM_INST, 0, GPIO_motor_PWM_C3_IDX);
        DL_TimerA_setCaptureCompareValue(motor_PWM_INST, pwm_ticks, GPIO_motor_PWM_C2_IDX);
    } else {
        DL_TimerA_setCaptureCompareValue(motor_PWM_INST, pwm_ticks, GPIO_motor_PWM_C3_IDX);
        DL_TimerA_setCaptureCompareValue(motor_PWM_INST, 0, GPIO_motor_PWM_C2_IDX);
    }
#else
    if (reverse) {
        DL_TimerG_setCaptureCompareValue(MotorLeft_INST, pwm_ticks, GPIO_MotorLeft_C0_IDX);
        DL_TimerG_setCaptureCompareValue(MotorLeft_INST, 0, GPIO_MotorLeft_C1_IDX);
    } else {
        DL_TimerG_setCaptureCompareValue(MotorLeft_INST, 0, GPIO_MotorLeft_C0_IDX);
        DL_TimerG_setCaptureCompareValue(MotorLeft_INST, pwm_ticks, GPIO_MotorLeft_C1_IDX);
    }
#endif
}

static void nm_write_right(uint16_t pwm_ticks, bool reverse)
{
#if NEWMOTOR_BACKEND_TIMA4CH
    if (reverse) {
        DL_TimerA_setCaptureCompareValue(motor_PWM_INST, 0, GPIO_motor_PWM_C0_IDX);
        DL_TimerA_setCaptureCompareValue(motor_PWM_INST, pwm_ticks, GPIO_motor_PWM_C1_IDX);
    } else {
        DL_TimerA_setCaptureCompareValue(motor_PWM_INST, pwm_ticks, GPIO_motor_PWM_C0_IDX);
        DL_TimerA_setCaptureCompareValue(motor_PWM_INST, 0, GPIO_motor_PWM_C1_IDX);
    }
#else
    if (reverse) {
        DL_TimerG_setCaptureCompareValue(MotorRight_INST, pwm_ticks, GPIO_MotorRight_C0_IDX);
        DL_TimerG_setCaptureCompareValue(MotorRight_INST, 0, GPIO_MotorRight_C1_IDX);
    } else {
        DL_TimerG_setCaptureCompareValue(MotorRight_INST, 0, GPIO_MotorRight_C0_IDX);
        DL_TimerG_setCaptureCompareValue(MotorRight_INST, pwm_ticks, GPIO_MotorRight_C1_IDX);
    }
#endif
}

void NewMotor_InitPwm(void)
{
#if NEWMOTOR_BACKEND_TIMA4CH
    DL_TimerA_startCounter(motor_PWM_INST);
#else
    DL_TimerG_startCounter(MotorLeft_INST);
    DL_TimerG_startCounter(MotorRight_INST);
#endif
    NewMotor_Stop(NEWMOTOR_STOP_COAST);
}

void NewMotor_SetWheelPwmTicksRaw(int16_t left_ticks, int16_t right_ticks)
{
    int16_t left = NewMotor_ClampSignedPwmTicks(left_ticks);
    int16_t right = NewMotor_ClampSignedPwmTicks(right_ticks);

    if (left == 0) {
        nm_write_left(0, false);
    } else {
        nm_write_left(nm_abs_i16(left), left < 0);
    }

    if (right == 0) {
        nm_write_right(0, false);
    } else {
        nm_write_right(nm_abs_i16(right), right < 0);
    }
}

void NewMotor_SetWheelPwmTicks(int16_t left_ticks, int16_t right_ticks)
{
    int16_t left = NewMotor_ApplyDeadzoneTicks(left_ticks);
    int16_t right = NewMotor_ApplyDeadzoneTicks(right_ticks);
    NewMotor_SetWheelPwmTicksRaw(left, right);
}

void NewMotor_Stop(NewMotor_StopMode mode)
{
    uint16_t stop_ticks = (mode == NEWMOTOR_STOP_BRAKE) ? NEWMOTOR_PWM_MAX_TICKS : 0;

#if NEWMOTOR_BACKEND_TIMA4CH
    DL_TimerA_setCaptureCompareValue(motor_PWM_INST, stop_ticks, GPIO_motor_PWM_C0_IDX);
    DL_TimerA_setCaptureCompareValue(motor_PWM_INST, stop_ticks, GPIO_motor_PWM_C1_IDX);
    DL_TimerA_setCaptureCompareValue(motor_PWM_INST, stop_ticks, GPIO_motor_PWM_C2_IDX);
    DL_TimerA_setCaptureCompareValue(motor_PWM_INST, stop_ticks, GPIO_motor_PWM_C3_IDX);
#else
    DL_TimerG_setCaptureCompareValue(MotorLeft_INST, stop_ticks, GPIO_MotorLeft_C0_IDX);
    DL_TimerG_setCaptureCompareValue(MotorLeft_INST, stop_ticks, GPIO_MotorLeft_C1_IDX);
    DL_TimerG_setCaptureCompareValue(MotorRight_INST, stop_ticks, GPIO_MotorRight_C0_IDX);
    DL_TimerG_setCaptureCompareValue(MotorRight_INST, stop_ticks, GPIO_MotorRight_C1_IDX);
#endif
}

float NewMotor_EncoderDeltaToDistanceMm(int32_t delta_counts)
{
    float distance_per_count_mm = NEWMOTOR_WHEEL_CIRCUMFERENCE_MM / NEWMOTOR_ENCODER_COUNTS_PER_WHEEL_REV;
    return ((float)delta_counts) * distance_per_count_mm;
}

float NewMotor_EncoderDeltaToWheelSpeedMmps(int32_t delta_counts, float sample_period_s)
{
    if (sample_period_s <= 0.0f) {
        return 0.0f;
    }
    return NewMotor_EncoderDeltaToDistanceMm(delta_counts) / sample_period_s;
}

float NewMotor_LeftRightToLinearSpeedMmps(float left_mmps, float right_mmps)
{
    return 0.5f * (left_mmps + right_mmps);
}

float NewMotor_LeftRightToYawRateRadps(float left_mmps, float right_mmps)
{
    if (NEWMOTOR_WHEEL_BASE_MM <= 0.0f) {
        return 0.0f;
    }
    return (right_mmps - left_mmps) / NEWMOTOR_WHEEL_BASE_MM;
}

#if NEWMOTOR_ENABLE_BSP_COMPAT_API
void Init_Motor_PWM(void)
{
    NewMotor_InitPwm();
}

void PWM_Control_Car(int16_t L_motor_speed, int16_t R_motor_speed)
{
    NewMotor_SetWheelPwmTicks(L_motor_speed, R_motor_speed);
}

void Motor_Stop(uint8_t brake)
{
    NewMotor_Stop(brake ? NEWMOTOR_STOP_BRAKE : NEWMOTOR_STOP_COAST);
}

void L1_control(uint16_t motor_speed, uint8_t dir)
{
    int16_t signed_pwm = dir ? -(int16_t)motor_speed : (int16_t)motor_speed;
    int16_t pwm = NewMotor_ClampSignedPwmTicks(signed_pwm);

    if (pwm == 0) {
        nm_write_left(0, false);
    } else {
        nm_write_left(nm_abs_i16(pwm), pwm < 0);
    }
}

void R1_control(uint16_t motor_speed, uint8_t dir)
{
    int16_t signed_pwm = dir ? -(int16_t)motor_speed : (int16_t)motor_speed;
    int16_t pwm = NewMotor_ClampSignedPwmTicks(signed_pwm);

    if (pwm == 0) {
        nm_write_right(0, false);
    } else {
        nm_write_right(nm_abs_i16(pwm), pwm < 0);
    }
}
#endif

#if NEWMOTOR_ENABLE_OLD_MOTOR_API
void Motor_Init(void)
{
    NewMotor_InitPwm();
}

void Motor_SetSpeed(int16_t left_speed, int16_t right_speed)
{
    NewMotor_SetWheelPwmTicks(left_speed, right_speed);
}

void Motor_Brake(void)
{
    NewMotor_Stop(NEWMOTOR_STOP_BRAKE);
}
#endif
