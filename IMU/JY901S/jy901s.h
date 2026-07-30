/**
 * @file    jy901s.h
 * @brief   JY901S / WT901 compatible I2C driver for TI MSPM0G3507.
 *
 * Wiring:
 *   JY901S SDA  --> same SDA as MPU6050 (I2C_mpu6050 / I2C0 PA28)
 *   JY901S SCL  --> same SCL as MPU6050 (I2C_mpu6050 / I2C0 PA31)
 *   JY901S VCC  --> 3.3V or 5V according to module label
 *   JY901S GND  --> GND
 *
 * The module uses WIT standard register protocol. Multi-byte sensor values are
 * little-endian signed 16-bit values.
 */

#ifndef JY901S_H_
#define JY901S_H_

#include "../imu_data.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

/* Use the same I2C controller and pins as MPU6050. */
#ifdef I2C_mpu6050_INST
#define JY901S_I2C_INST I2C_mpu6050_INST
#else
#define JY901S_I2C_INST I2C0
#endif

/* Default WIT I2C 7-bit address. */
#define JY901S_ADDR (0x50u)

/* Common WIT register map. */
#define JY901S_REG_SAVE       (0x00u)
#define JY901S_REG_CALSW      (0x01u)
#define JY901S_REG_RSW        (0x02u)
#define JY901S_REG_RRATE      (0x03u)
#define JY901S_REG_BAUD       (0x04u)
#define JY901S_REG_AXOFFSET   (0x05u)
#define JY901S_REG_AYOFFSET   (0x06u)
#define JY901S_REG_AZOFFSET   (0x07u)
#define JY901S_REG_GXOFFSET   (0x08u)
#define JY901S_REG_GYOFFSET   (0x09u)
#define JY901S_REG_GZOFFSET   (0x0Au)
#define JY901S_REG_HXOFFSET   (0x0Bu)
#define JY901S_REG_HYOFFSET   (0x0Cu)
#define JY901S_REG_HZOFFSET   (0x0Du)
#define JY901S_REG_D0MODE     (0x0Eu)
#define JY901S_REG_D1MODE     (0x0Fu)
#define JY901S_REG_D2MODE     (0x10u)
#define JY901S_REG_D3MODE     (0x11u)
#define JY901S_REG_D0PWMH     (0x12u)
#define JY901S_REG_D1PWMH     (0x13u)
#define JY901S_REG_D2PWMH     (0x14u)
#define JY901S_REG_D3PWMH     (0x15u)
#define JY901S_REG_D0PWMT     (0x16u)
#define JY901S_REG_D1PWMT     (0x17u)
#define JY901S_REG_D2PWMT     (0x18u)
#define JY901S_REG_D3PWMT     (0x19u)
#define JY901S_REG_IICADDR    (0x1Au)
#define JY901S_REG_LEDOFF     (0x1Bu)
#define JY901S_REG_GPSBAUD    (0x1Cu)
#define JY901S_REG_KEY        (0x69u)

#define JY901S_REG_YYMM       (0x30u)
#define JY901S_REG_DDHH       (0x31u)
#define JY901S_REG_MMSS       (0x32u)
#define JY901S_REG_MS         (0x33u)
#define JY901S_REG_AX         (0x34u)
#define JY901S_REG_AY         (0x35u)
#define JY901S_REG_AZ         (0x36u)
#define JY901S_REG_GX         (0x37u)
#define JY901S_REG_GY         (0x38u)
#define JY901S_REG_GZ         (0x39u)
#define JY901S_REG_HX         (0x3Au)
#define JY901S_REG_HY         (0x3Bu)
#define JY901S_REG_HZ         (0x3Cu)
#define JY901S_REG_ROLL       (0x3Du)
#define JY901S_REG_PITCH      (0x3Eu)
#define JY901S_REG_YAW        (0x3Fu)
#define JY901S_REG_TEMP       (0x40u)

/* CALSW command values. */
#define JY901S_CALSW_ZERO_YAW (0x0004u)

/* Write KEY before changing configuration or starting calibration commands. */
#define JY901S_KEY_UNLOCK     (0xB588u)

/* SAVE command values. */
#define JY901S_SAVE_CONFIG     (0x0000u)

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} JY901S_VectorRaw_t;

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t hx;
    int16_t hy;
    int16_t hz;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t temp;
} JY901S_RawData_t;

bool JY901S_Init(void);
bool JY901S_IsConnected(void);
bool JY901S_ProbeAddress(uint8_t addr);
uint8_t JY901S_ScanFirstAddress(void);

bool JY901S_ReadRegister(uint8_t reg, uint16_t *value);
bool JY901S_WriteRegister(uint8_t reg, uint16_t value);
bool JY901S_ReadBytes(uint8_t startReg, uint8_t *buffer, uint8_t length);
bool JY901S_ZeroYaw(void);

bool JY901S_ReadRaw(JY901S_RawData_t *out);
bool JY901S_ReadAll(JY901S_Data_t *out);
bool JY901S_ReadAccelRaw(JY901S_VectorRaw_t *out);
bool JY901S_ReadGyroRaw(JY901S_VectorRaw_t *out);
bool JY901S_ReadMagRaw(JY901S_VectorRaw_t *out);
bool JY901S_ReadAngleRaw(JY901S_VectorRaw_t *out);
/*
 * Set the current X/Y acceleration output to zero and save the new offsets.
 * Keep the sensor horizontal and completely still while calling this function.
 */
bool JY901S_ZeroAxAy(void);

void JY901S_ConvertRaw(const JY901S_RawData_t *raw, JY901S_Data_t *out);

#endif /* JY901S_H_ */
