#ifndef NEWMOTOR_DRIVER_H
#define NEWMOTOR_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------
 * PWM unit: timer ticks
 * Sign convention: +forward, -reverse
 * ----------------------------- */
#ifndef NEWMOTOR_PWM_MAX_TICKS
#define NEWMOTOR_PWM_MAX_TICKS (1000)
#endif

#ifndef NEWMOTOR_PWM_DEADZONE_TICKS
#define NEWMOTOR_PWM_DEADZONE_TICKS (75)
#endif

/* -----------------------------
 * Mechanical parameters (units are explicit)
 * ----------------------------- */
#ifndef NEWMOTOR_WHEEL_DIAMETER_MM
#define NEWMOTOR_WHEEL_DIAMETER_MM (66.0f)
#endif

#ifndef NEWMOTOR_WHEEL_BASE_MM
#define NEWMOTOR_WHEEL_BASE_MM (90.0f)
#endif

#ifndef NEWMOTOR_GEAR_RATIO
#define NEWMOTOR_GEAR_RATIO (28.0f)
#endif

/* Encoder cycles per MOTOR shaft revolution before quadrature x4. */
#ifndef NEWMOTOR_ENCODER_CPR_MOTOR
#define NEWMOTOR_ENCODER_CPR_MOTOR (13.0f)
#endif

/* User stated interrupt should be quadrature x4. */
#ifndef NEWMOTOR_ENCODER_QUADRATURE_MULTIPLIER
#define NEWMOTOR_ENCODER_QUADRATURE_MULTIPLIER (4.0f)
#endif

#define NEWMOTOR_PI (3.14159265359f)
#define NEWMOTOR_WHEEL_CIRCUMFERENCE_MM (NEWMOTOR_PI * NEWMOTOR_WHEEL_DIAMETER_MM)
#define NEWMOTOR_ENCODER_COUNTS_PER_WHEEL_REV \
    (NEWMOTOR_ENCODER_CPR_MOTOR * NEWMOTOR_ENCODER_QUADRATURE_MULTIPLIER * NEWMOTOR_GEAR_RATIO)

typedef enum NewMotor_StopMode {
    NEWMOTOR_STOP_COAST = 0,
    NEWMOTOR_STOP_BRAKE = 1
} NewMotor_StopMode;

/* Basic driver API */
void NewMotor_InitPwm(void);
void NewMotor_SetWheelPwmTicksRaw(int16_t left_ticks, int16_t right_ticks);
void NewMotor_SetWheelPwmTicks(int16_t left_ticks, int16_t right_ticks);
void NewMotor_Stop(NewMotor_StopMode mode);

/* Helpers (unit: PWM ticks) */
int16_t NewMotor_ClampSignedPwmTicks(int16_t ticks);
int16_t NewMotor_ApplyDeadzoneTicks(int16_t ticks);

/* Encoder and kinematics helpers */
float NewMotor_EncoderDeltaToDistanceMm(int32_t delta_counts);
float NewMotor_EncoderDeltaToWheelSpeedMmps(int32_t delta_counts, float sample_period_s);
float NewMotor_LeftRightToLinearSpeedMmps(float left_mmps, float right_mmps);
float NewMotor_LeftRightToYawRateRadps(float left_mmps, float right_mmps);

/*
 * Optional compatibility wrappers.
 * Enable only when replacing old driver files to avoid duplicate symbols.
 */
#ifndef NEWMOTOR_ENABLE_BSP_COMPAT_API
#define NEWMOTOR_ENABLE_BSP_COMPAT_API (0)
#endif

#if NEWMOTOR_ENABLE_BSP_COMPAT_API
void Init_Motor_PWM(void);
void PWM_Control_Car(int16_t L_motor_speed, int16_t R_motor_speed);
void Motor_Stop(uint8_t brake);
void L1_control(uint16_t motor_speed, uint8_t dir);
void R1_control(uint16_t motor_speed, uint8_t dir);
#endif

#ifndef NEWMOTOR_ENABLE_OLD_MOTOR_API
#define NEWMOTOR_ENABLE_OLD_MOTOR_API (0)
#endif

#if NEWMOTOR_ENABLE_OLD_MOTOR_API
void Motor_Init(void);
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed);
void Motor_Brake(void);
#endif

#ifdef __cplusplus
}
#endif

#endif
