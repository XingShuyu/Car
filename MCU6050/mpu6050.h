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
/* #define MPU6050_ADDR         (0x69u)   // AD0 = VCC */

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

/**
 * @brief  Initialize MPU6050 with default settings
 *         (±2g, ±250°/s, DLPF=0, SampleRate divider=0)
 * @return true if successful, false if device not found
 */
bool MPU6050_Init(void);

/**
 * @brief  Initialize MPU6050 with custom configuration
 */
bool MPU6050_InitWithConfig(const MPU6050_Config_t *cfg);

/**
 * @brief  Read raw accelerometer data (16-bit signed)
 */
bool MPU6050_ReadAccelRaw(MPU6050_RawData_t *out);

/**
 * @brief  Read raw gyroscope data (16-bit signed)
 */
bool MPU6050_ReadGyroRaw(MPU6050_RawData_t *out);

/**
 * @brief  Read raw temperature register value
 */
bool MPU6050_ReadTempRaw(int16_t *out);

/**
 * @brief  Read all sensor data converted to physical units
 */
bool MPU6050_ReadAll(MPU6050_Data_t *out);

/**
 * @brief  Software-reset the MPU6050
 */
bool MPU6050_Reset(void);

/**
 * @brief  Read WHO_AM_I register (should return 0x68)
 */
bool MPU6050_WhoAmI(uint8_t *id);

/**
 * @brief  Set gyroscope full-scale range
 */
bool MPU6050_SetGyroFS(MPU6050_GyroFS_t fs);

/**
 * @brief  Set accelerometer full-scale range
 */
bool MPU6050_SetAccelFS(MPU6050_AccelFS_t fs);

#endif /* MPU6050_H_ */
