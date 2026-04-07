/**
 * @file    mpu6050.h
 * @brief   MPU6050 Driver for TI MSPM0G3507 (TI CCS)
 * @details Uses MSPM0 DriverLib I2C peripheral (I2C1)
 *
 * Wiring:
 *   MPU6050 SDA  --> MSPM0G3507 PB3 (I2C1_SDA)
 *   MPU6050 SCL  --> MSPM0G3507 PB2 (I2C1_SCL)
 *   MPU6050 VCC  --> 3.3V
 *   MPU6050 GND  --> GND
 *   MPU6050 AD0  --> GND  (I2C address = 0x68)
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/*  I2C Instance (change to I2C0 if needed)                            */
/* ------------------------------------------------------------------ */
#define MPU6050_I2C_INST        I2C0

/* ------------------------------------------------------------------ */
/*  MPU6050 I2C Address                                                */
/* ------------------------------------------------------------------ */
#define MPU6050_ADDR            (0x68u)   /* AD0 = GND */

/* ------------------------------------------------------------------ */
/*  MPU6050 Register Map                                               */
/* ------------------------------------------------------------------ */
#define MPU6050_REG_SELF_TEST_X     0x0Du
#define MPU6050_REG_SELF_TEST_Y     0x0Eu
#define MPU6050_REG_SELF_TEST_Z     0x0Fu
#define MPU6050_REG_SELF_TEST_A     0x10u
#define MPU6050_REG_SMPLRT_DIV      0x19u
#define MPU6050_REG_CONFIG          0x1Au
#define MPU6050_REG_GYRO_CONFIG     0x1Bu
#define MPU6050_REG_ACCEL_CONFIG    0x1Cu
#define MPU6050_REG_FIFO_EN         0x23u
#define MPU6050_REG_INT_PIN_CFG     0x37u
#define MPU6050_REG_INT_ENABLE      0x38u
#define MPU6050_REG_INT_STATUS      0x3Au
#define MPU6050_REG_ACCEL_XOUT_H    0x3Bu
#define MPU6050_REG_ACCEL_XOUT_L    0x3Cu
#define MPU6050_REG_ACCEL_YOUT_H    0x3Du
#define MPU6050_REG_ACCEL_YOUT_L    0x3Eu
#define MPU6050_REG_ACCEL_ZOUT_H    0x3Fu
#define MPU6050_REG_ACCEL_ZOUT_L    0x40u
#define MPU6050_REG_TEMP_OUT_H      0x41u
#define MPU6050_REG_TEMP_OUT_L      0x42u
#define MPU6050_REG_GYRO_XOUT_H     0x43u
#define MPU6050_REG_GYRO_XOUT_L     0x44u
#define MPU6050_REG_GYRO_YOUT_H     0x45u
#define MPU6050_REG_GYRO_YOUT_L     0x46u
#define MPU6050_REG_GYRO_ZOUT_H     0x47u
#define MPU6050_REG_GYRO_ZOUT_L     0x48u
#define MPU6050_REG_USER_CTRL       0x6Au
#define MPU6050_REG_PWR_MGMT_1      0x6Bu
#define MPU6050_REG_PWR_MGMT_2      0x6Cu
#define MPU6050_REG_WHO_AM_I        0x75u

/* ------------------------------------------------------------------ */
/*  Configuration Values                                               */
/* ------------------------------------------------------------------ */
#define MPU6050_WHO_AM_I_VAL        0x68u

/* Gyroscope full-scale range */
typedef enum {
    MPU6050_GYRO_FS_250DPS  = 0x00,   /* ±250  °/s,  131   LSB/(°/s) */
    MPU6050_GYRO_FS_500DPS  = 0x08,   /* ±500  °/s,  65.5  LSB/(°/s) */
    MPU6050_GYRO_FS_1000DPS = 0x10,   /* ±1000 °/s,  32.8  LSB/(°/s) */
    MPU6050_GYRO_FS_2000DPS = 0x18    /* ±2000 °/s,  16.4  LSB/(°/s) */
} MPU6050_GyroFS_t;

/* Accelerometer full-scale range */
typedef enum {
    MPU6050_ACCEL_FS_2G  = 0x00,      /* ±2g,  16384 LSB/g */
    MPU6050_ACCEL_FS_4G  = 0x08,      /* ±4g,   8192 LSB/g */
    MPU6050_ACCEL_FS_8G  = 0x10,      /* ±8g,   4096 LSB/g */
    MPU6050_ACCEL_FS_16G = 0x18       /* ±16g,  2048 LSB/g */
} MPU6050_AccelFS_t;

/* ------------------------------------------------------------------ */
/*  Data Structures                                                    */
/* ------------------------------------------------------------------ */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} MPU6050_RawData_t;

typedef struct {
    float ax;   /* Acceleration X [g]    */
    float ay;   /* Acceleration Y [g]    */
    float az;   /* Acceleration Z [g]    */
    float gx;   /* Angular velocity X [°/s] */
    float gy;   /* Angular velocity Y [°/s] */
    float gz;   /* Angular velocity Z [°/s] */
    float temp; /* Temperature [°C]      */
} MPU6050_Data_t;

typedef struct {
    MPU6050_GyroFS_t  gyroFS;
    MPU6050_AccelFS_t accelFS;
    uint8_t           dlpfConfig;   /* Digital Low Pass Filter 0~6 */
    uint8_t           sampleRateDiv;/* Sample Rate Divider         */
} MPU6050_Config_t;

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

/* 基本初始化与配置 */
bool MPU6050_Init(void);
bool MPU6050_InitWithConfig(const MPU6050_Config_t *cfg);
bool MPU6050_Reset(void);
bool MPU6050_WhoAmI(uint8_t *id);
bool MPU6050_SetGyroFS(MPU6050_GyroFS_t fs);
bool MPU6050_SetAccelFS(MPU6050_AccelFS_t fs);

/* 原始数据读取 */
bool MPU6050_ReadAccelRaw(MPU6050_RawData_t *out);
bool MPU6050_ReadGyroRaw(MPU6050_RawData_t *out);
bool MPU6050_ReadTempRaw(int16_t *out);
bool MPU6050_ReadAll(MPU6050_Data_t *out);

/* ---------- 新增零偏校准相关函数 ---------- */

/**
 * @brief 校准陀螺仪零偏（静止状态下调用）
 * @param samples 采样次数，建议 500~2000
 * @return true 成功，false 失败
 * @note 校准期间必须保持 MPU6050 绝对静止
 */
bool MPU6050_CalibrateGyro(uint16_t samples);

/**
 * @brief 获取当前陀螺仪零偏值（单位：°/s）
 * @param x 输出 X 轴零偏
 * @param y 输出 Y 轴零偏
 * @param z 输出 Z 轴零偏
 */
void MPU6050_GetGyroZero(float *x, float *y, float *z);

/**
 * @brief 手动设置陀螺仪零偏值
 * @param x X 轴零偏
 * @param y Y 轴零偏
 * @param z Z 轴零偏
 */
void MPU6050_SetGyroZero(float x, float y, float z);

/**
 * @brief 读取所有数据，并返回已减去陀螺仪零偏的角速度值
 * @param out 输出数据结构（加速度、温度、校准后的角速度）
 * @return true 成功，false 失败
 */
bool MPU6050_ReadAllCalibrated(MPU6050_Data_t *out);

#endif /* MPU6050_H_ */