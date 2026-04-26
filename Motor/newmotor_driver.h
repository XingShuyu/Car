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
#if defined(motor_PWM_INST)
#define NEWMOTOR_PWM_MAX_TICKS (1000)
#elif defined(MotorLeft_INST) && defined(MotorRight_INST)
#define NEWMOTOR_PWM_MAX_TICKS (2000)
#else
#define NEWMOTOR_PWM_MAX_TICKS (1000)
#endif
#endif

/*
 * PWM period register value (ARR/load).
 * Use this value for full brake output.
 */
#ifndef NEWMOTOR_PWM_PERIOD_TICKS
#if defined(motor_PWM_INST)
#define NEWMOTOR_PWM_PERIOD_TICKS (1000)
#elif defined(MotorLeft_INST) && defined(MotorRight_INST)
#define NEWMOTOR_PWM_PERIOD_TICKS (3200)
#else
#define NEWMOTOR_PWM_PERIOD_TICKS (1000)
#endif
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
#define NEWMOTOR_WHEEL_BASE_MM (45.0f)
#endif

#ifndef NEWMOTOR_GEAR_RATIO
#define NEWMOTOR_GEAR_RATIO (28.0f)
#endif

/* Encoder cycles per MOTOR shaft revolution before quadrature x4. */
#ifndef NEWMOTOR_ENCODER_CPR_MOTOR
#define NEWMOTOR_ENCODER_CPR_MOTOR (500.0f)
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
/**
 * @brief 初始化电机 PWM 驱动并将电机置于停止状态。
 *
 * @param 无。
 */
void NewMotor_InitPwm(void);

/**
 * @brief 直接按原始 PWM ticks 设置左右轮输出，不做死区补偿。
 *
 * @param left_ticks 左轮 PWM 比较值（单位：ticks，正数前进，负数后退）。
 * @param right_ticks 右轮 PWM 比较值（单位：ticks，正数前进，负数后退）。
 */
void NewMotor_SetWheelPwmTicksRaw(int16_t left_ticks, int16_t right_ticks);

/**
 * @brief 设置左右轮 PWM 输出，内部会做限幅与死区补偿。
 *
 * @param left_ticks 左轮 PWM 比较值（单位：ticks，正数前进，负数后退）。
 * @param right_ticks 右轮 PWM 比较值（单位：ticks，正数前进，负数后退）。
 */
void NewMotor_SetWheelPwmTicks(int16_t left_ticks, int16_t right_ticks);

/**
 * @brief 停止电机，可选择滑行停止或电子刹车。
 *
 * @param mode 停止模式：NEWMOTOR_STOP_COAST 或 NEWMOTOR_STOP_BRAKE。
 */
void NewMotor_Stop(NewMotor_StopMode mode);

/* Helpers (unit: PWM ticks) */
/**
 * @brief 将有符号 PWM 值限制到 [-NEWMOTOR_PWM_MAX_TICKS, NEWMOTOR_PWM_MAX_TICKS]。
 *
 * @param ticks 输入 PWM 值（单位：ticks，正负表示方向）。
 * @return 限幅后的 PWM 值（单位：ticks）。
 */
int16_t NewMotor_ClampSignedPwmTicks(int16_t ticks);

/**
 * @brief 在限幅基础上叠加死区补偿，避免低占空比无法起转。
 *
 * @param ticks 输入 PWM 值（单位：ticks，正负表示方向）。
 * @return 死区补偿后的 PWM 值（单位：ticks）。
 */
int16_t NewMotor_ApplyDeadzoneTicks(int16_t ticks);

/* Encoder and kinematics helpers */
/**
 * @brief 将编码器增量计数换算为车轮位移。
 *
 * @param delta_counts 采样周期内编码器增量计数（单位：count）。
 * @return 对应位移（单位：mm）。
 */
float NewMotor_EncoderDeltaToDistanceMm(int32_t delta_counts);

/**
 * @brief 将编码器增量计数换算为车轮线速度。
 *
 * @param delta_counts 采样周期内编码器增量计数（单位：count）。
 * @param sample_period_s 采样周期（单位：s）。
 * @return 车轮线速度（单位：mm/s）。
 */
float NewMotor_EncoderDeltaToWheelSpeedMmps(int32_t delta_counts, float sample_period_s);

/**
 * @brief 由左右轮线速度计算车体线速度（差速底盘模型）。
 *
 * @param left_mmps 左轮线速度（单位：mm/s）。
 * @param right_mmps 右轮线速度（单位：mm/s）。
 * @return 车体前向线速度（单位：mm/s）。
 */
float NewMotor_LeftRightToLinearSpeedMmps(float left_mmps, float right_mmps);

/**
 * @brief 由左右轮线速度计算车体偏航角速度（差速底盘模型）。
 *
 * @param left_mmps 左轮线速度（单位：mm/s）。
 * @param right_mmps 右轮线速度（单位：mm/s）。
 * @return 车体偏航角速度（单位：rad/s）。
 */
float NewMotor_LeftRightToYawRateRadps(float left_mmps, float right_mmps);

/*
 * Optional compatibility wrappers.
 * Enable only when replacing old driver files to avoid duplicate symbols.
 */
#ifndef NEWMOTOR_ENABLE_BSP_COMPAT_API
#define NEWMOTOR_ENABLE_BSP_COMPAT_API (0)
#endif

#if NEWMOTOR_ENABLE_BSP_COMPAT_API
/**
 * @brief 兼容旧 BSP 接口：初始化电机 PWM。
 *
 * @param 无。
 */
void Init_Motor_PWM(void);

/**
 * @brief 兼容旧 BSP 接口：设置左右电机 PWM。
 *
 * @param L_motor_speed 左轮 PWM（单位：ticks，正前负后）。
 * @param R_motor_speed 右轮 PWM（单位：ticks，正前负后）。
 */
void PWM_Control_Car(int16_t L_motor_speed, int16_t R_motor_speed);

/**
 * @brief 兼容旧 BSP 接口：停止电机。
 *
 * @param brake 停止模式，0=滑行停止，非0=电子刹车。
 */
void Motor_Stop(uint8_t brake);

/**
 * @brief 兼容旧 BSP 接口：单独控制左电机。
 *
 * @param motor_speed 左电机 PWM 绝对值（单位：ticks）。
 * @param dir 方向，0=前进，非0=后退。
 */
void L1_control(uint16_t motor_speed, uint8_t dir);

/**
 * @brief 兼容旧 BSP 接口：单独控制右电机。
 *
 * @param motor_speed 右电机 PWM 绝对值（单位：ticks）。
 * @param dir 方向，0=前进，非0=后退。
 */
void R1_control(uint16_t motor_speed, uint8_t dir);
#endif

#ifndef NEWMOTOR_ENABLE_OLD_MOTOR_API
#define NEWMOTOR_ENABLE_OLD_MOTOR_API (0)
#endif

#if NEWMOTOR_ENABLE_OLD_MOTOR_API
/**
 * @brief 兼容旧 Motor 接口：初始化电机。
 *
 * @param 无。
 */
void Motor_Init(void);

/**
 * @brief 兼容旧 Motor 接口：设置左右电机速度。
 *
 * @param left_speed 左轮 PWM（单位：ticks，正前负后）。
 * @param right_speed 右轮 PWM（单位：ticks，正前负后）。
 */
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed);

/**
 * @brief 兼容旧 Motor 接口：电子刹车停止。
 *
 * @param 无。
 */
void Motor_Brake(void);
#endif

#ifdef __cplusplus
}
#endif

#endif
