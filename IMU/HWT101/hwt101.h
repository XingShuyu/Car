/**
 * @file    hwt101.h
 * @brief   HWT101 single-axis WIT I2C driver for TI MSPM0G3507.
 *
 * Wiring:
 *   HWT101 SDA  --> same SDA as JY901S/MPU6050 (I2C_mpu6050 / I2C0 PA28)
 *   HWT101 SCL  --> same SCL as JY901S/MPU6050 (I2C_mpu6050 / I2C0 PA31)
 *   HWT101 VCC  --> module supply according to label
 *   HWT101 GND  --> GND
 *
 * The HWT101 uses the WIT standard register protocol. Register values are
 * little-endian signed 16-bit values on I2C.
 */

#ifndef HWT101_H_
#define HWT101_H_

#include "../imu_data.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef I2C_mpu6050_INST
#define HWT101_I2C_INST I2C_mpu6050_INST
#else
#define HWT101_I2C_INST I2C0
#endif

/* Default WIT 7-bit I2C address. */
#define HWT101_ADDR (0x50u)

/* HWT101 / WIT register map used by this driver. */
#define HWT101_REG_SAVE       (0x00u)
#define HWT101_REG_CALSW      (0x01u)
#define HWT101_REG_RSW        (0x02u)
#define HWT101_REG_RRATE      (0x03u)
#define HWT101_REG_BAUD       (0x04u)
#define HWT101_REG_IICADDR    (0x1Au)
#define HWT101_REG_READADDR   (0x27u)
#define HWT101_REG_VERSION    (0x2Eu)
#define HWT101_REG_AX         (0x34u)
#define HWT101_REG_AY         (0x35u)
#define HWT101_REG_AZ         (0x36u)
#define HWT101_REG_GY         (0x38u)
#define HWT101_REG_GZ         (0x39u)
#define HWT101_REG_YAW        (0x3Fu)
#define HWT101_REG_REFYAW     (0x65u)
#define HWT101_REG_MODDELAY   (0x74u)
#define HWT101_REG_YAWOFFSET  (0x75u)
#define HWT101_REG_CALIYAW    (0x76u)
#define HWT101_REG_KEY        (0x69u)

#define HWT101_KEY_UNLOCK     (0xB588u)
#define HWT101_CALSW_ZERO_YAW (0x0004u)
#define HWT101_CALIYAW_CLEAR  (0x0001u)
#define HWT101_CALIYAW_NORMAL (0x0000u)

typedef struct {
    int16_t gy;
    int16_t gz;
    int16_t yaw;
    int16_t version;
} HWT101_RawData_t;

typedef struct {
    int16_t y;
    int16_t z;
} HWT101_GyroRaw_t;

bool HWT101_Init(void);
bool HWT101_IsConnected(void);
bool HWT101_ProbeAddress(uint8_t addr);
uint8_t HWT101_ScanFirstAddress(void);

bool HWT101_ReadRegister(uint8_t reg, uint16_t *value);
bool HWT101_WriteRegister(uint8_t reg, uint16_t value);
bool HWT101_ReadBytes(uint8_t startReg, uint8_t *buffer, uint8_t length);
bool HWT101_ZeroYaw(void);

bool HWT101_ReadRaw(HWT101_RawData_t *out);
bool HWT101_ReadGyroRaw(HWT101_GyroRaw_t *out);
bool HWT101_ReadYawRaw(int16_t *out);
bool HWT101_ReadAll(IMU_Data_t *out);

void HWT101_ConvertRaw(const HWT101_RawData_t *raw, IMU_Data_t *out);

#endif /* HWT101_H_ */
