/**
 * @file    mpu6050.c
 * @brief   MPU6050 Driver Implementation for TI MSPM0G3507
 *
 * Depends on:
 *  - MSPM0 DriverLib (driverlib/dl_i2c.h)
 *  - SysConfig-generated ti_msp_dl_config.h / ti_msp_dl_config.c
 *    with I2C1 configured as Master, ~400 kHz, on PB2(SCL)/PB3(SDA)
 */

#include "mpu6050.h"
#include "ti/driverlib/dl_i2c.h"
#include <stddef.h>

/* ------------------------------------------------------------------ */
/*  Timeout for I2C polling loops  (adjust to your clock frequency)   */
/* ------------------------------------------------------------------ */
#define I2C_TIMEOUT_MS   (1000u)

/* ------------------------------------------------------------------ */
/*  Internal scale factors (updated when FS range changes)            */
/* ------------------------------------------------------------------ */
static float s_accelScale = 1.0f / 16384.0f;  /* ±2g  default */
static float s_gyroScale  = 1.0f / 131.0f;    /* ±250°/s default */

/* ---------- 新增：陀螺仪零偏存储（单位：°/s） ---------- */
static float s_gyroZeroX = 0.0f;
static float s_gyroZeroY = 0.0f;
static float s_gyroZeroZ = 0.0f;

/* ================================================================== */
/*  Low-level I2C helpers (保持不变)                                   */
/* ================================================================== */

static bool i2c_wait_idle(void)
{
    uint32_t timeout = I2C_TIMEOUT_MS * 1000u;
    while (DL_I2C_getControllerStatus(MPU6050_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--timeout == 0u) return false;
    }
    return true;
}

static bool i2c_wait_done(void)
{
    uint32_t timeout = I2C_TIMEOUT_MS * 1000u;
    while (!DL_I2C_getRawInterruptStatus(
                MPU6050_I2C_INST,
                DL_I2C_INTERRUPT_CONTROLLER_STOP |
                DL_I2C_INTERRUPT_CONTROLLER_NACK)) {
        if (--timeout == 0u) return false;
    }
    if (DL_I2C_getRawInterruptStatus(MPU6050_I2C_INST,
            DL_I2C_INTERRUPT_CONTROLLER_NACK)) {
        DL_I2C_clearInterruptStatus(MPU6050_I2C_INST,
            DL_I2C_INTERRUPT_CONTROLLER_NACK |
            DL_I2C_INTERRUPT_CONTROLLER_STOP);
        return false;
    }
    DL_I2C_clearInterruptStatus(MPU6050_I2C_INST,
        DL_I2C_INTERRUPT_CONTROLLER_STOP);
    return true;
}

static bool mpu6050_write_reg(uint8_t reg, uint8_t val)
{
    if (!i2c_wait_idle()) return false;

    DL_I2C_clearInterruptStatus(MPU6050_I2C_INST,
        DL_I2C_INTERRUPT_CONTROLLER_NACK |
        DL_I2C_INTERRUPT_CONTROLLER_STOP);

    DL_I2C_flushControllerTXFIFO(MPU6050_I2C_INST);
    DL_I2C_fillControllerTXFIFO(MPU6050_I2C_INST, &reg, 1);
    DL_I2C_fillControllerTXFIFO(MPU6050_I2C_INST, &val, 1);

    DL_I2C_startControllerTransfer(
        MPU6050_I2C_INST,
        MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        2u);

    return i2c_wait_done();
}

static bool mpu6050_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (!i2c_wait_idle()) return false;

    DL_I2C_clearInterruptStatus(MPU6050_I2C_INST,
        DL_I2C_INTERRUPT_CONTROLLER_NACK |
        DL_I2C_INTERRUPT_CONTROLLER_STOP |
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);

    DL_I2C_flushControllerTXFIFO(MPU6050_I2C_INST);
    DL_I2C_fillControllerTXFIFO(MPU6050_I2C_INST, &reg, 1);

    DL_I2C_startControllerTransfer(
        MPU6050_I2C_INST,
        MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        1u);

    if (!i2c_wait_done()) return false;
    if (!i2c_wait_idle()) return false;

    DL_I2C_clearInterruptStatus(MPU6050_I2C_INST,
        DL_I2C_INTERRUPT_CONTROLLER_NACK |
        DL_I2C_INTERRUPT_CONTROLLER_STOP |
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);

    DL_I2C_flushControllerRXFIFO(MPU6050_I2C_INST);

    DL_I2C_startControllerTransfer(
        MPU6050_I2C_INST,
        MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX,
        (uint16_t)len);

    uint32_t timeout = I2C_TIMEOUT_MS * 1000u;
    for (uint8_t i = 0; i < len; i++) {
        while (DL_I2C_isControllerRXFIFOEmpty(MPU6050_I2C_INST)) {
            if (--timeout == 0u) return false;
        }
        buf[i] = DL_I2C_receiveControllerData(MPU6050_I2C_INST);
    }

    if (!i2c_wait_done()) return false;
    return true;
}

/* ================================================================== */
/*  Public API Implementation                                          */
/* ================================================================== */

bool MPU6050_WhoAmI(uint8_t *id)
{
    return mpu6050_read_regs(MPU6050_REG_WHO_AM_I, id, 1u);
}

bool MPU6050_Reset(void)
{
    if (!mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x80u)) return false;
    uint32_t delay = 100000u;
    while (delay--) { __NOP(); }
    return true;
}

bool MPU6050_SetGyroFS(MPU6050_GyroFS_t fs)
{
    if (!mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, (uint8_t)fs)) return false;
    switch (fs) {
        case MPU6050_GYRO_FS_250DPS:  s_gyroScale = 1.0f / 131.0f;  break;
        case MPU6050_GYRO_FS_500DPS:  s_gyroScale = 1.0f / 65.5f;   break;
        case MPU6050_GYRO_FS_1000DPS: s_gyroScale = 1.0f / 32.8f;   break;
        case MPU6050_GYRO_FS_2000DPS: s_gyroScale = 1.0f / 16.4f;   break;
        default: break;
    }
    return true;
}

bool MPU6050_SetAccelFS(MPU6050_AccelFS_t fs)
{
    if (!mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, (uint8_t)fs)) return false;
    switch (fs) {
        case MPU6050_ACCEL_FS_2G:  s_accelScale = 1.0f / 16384.0f; break;
        case MPU6050_ACCEL_FS_4G:  s_accelScale = 1.0f / 8192.0f;  break;
        case MPU6050_ACCEL_FS_8G:  s_accelScale = 1.0f / 4096.0f;  break;
        case MPU6050_ACCEL_FS_16G: s_accelScale = 1.0f / 2048.0f;  break;
        default: break;
    }
    return true;
}

bool MPU6050_Init(void)
{
    MPU6050_Config_t defaultCfg = {
        .gyroFS       = MPU6050_GYRO_FS_250DPS,
        .accelFS      = MPU6050_ACCEL_FS_2G,
        .dlpfConfig   = 0x01u,
        .sampleRateDiv= 0x00u
    };
    return MPU6050_InitWithConfig(&defaultCfg);
}

bool MPU6050_InitWithConfig(const MPU6050_Config_t *cfg)
{
    if (cfg == NULL) return false;

    uint8_t whoami = 0x00u;
    if (!MPU6050_WhoAmI(&whoami)) return false;
    if (whoami != MPU6050_WHO_AM_I_VAL) return false;

    if (!MPU6050_Reset()) return false;

    if (!mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x01u)) return false;

    if (!mpu6050_write_reg(MPU6050_REG_CONFIG, cfg->dlpfConfig & 0x07u))
        return false;

    if (!mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV, cfg->sampleRateDiv))
        return false;

    if (!MPU6050_SetGyroFS(cfg->gyroFS)) return false;
    if (!MPU6050_SetAccelFS(cfg->accelFS)) return false;

    /* 初始化零偏为 0 */
    s_gyroZeroX = s_gyroZeroY = s_gyroZeroZ = 0.0f;

    return true;
}

bool MPU6050_ReadAccelRaw(MPU6050_RawData_t *out)
{
    if (out == NULL) return false;
    uint8_t buf[6];
    if (!mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, buf, 6u)) return false;
    out->x = (int16_t)((buf[0] << 8) | buf[1]);
    out->y = (int16_t)((buf[2] << 8) | buf[3]);
    out->z = (int16_t)((buf[4] << 8) | buf[5]);
    return true;
}

bool MPU6050_ReadGyroRaw(MPU6050_RawData_t *out)
{
    if (out == NULL) return false;
    uint8_t buf[6];
    if (!mpu6050_read_regs(MPU6050_REG_GYRO_XOUT_H, buf, 6u)) return false;
    out->x = (int16_t)((buf[0] << 8) | buf[1]);
    out->y = (int16_t)((buf[2] << 8) | buf[3]);
    out->z = (int16_t)((buf[4] << 8) | buf[5]);
    if (out->z>-50 && out->z<50) {
        out->z = 0;
    }
    else {
        out->z = out->z*s_gyroScale;
    }
    return true;
}

bool MPU6050_ReadTempRaw(int16_t *out)
{
    if (out == NULL) return false;
    uint8_t buf[2];
    if (!mpu6050_read_regs(MPU6050_REG_TEMP_OUT_H, buf, 2u)) return false;
    *out = (int16_t)((buf[0] << 8) | buf[1]);
    return true;
}

bool MPU6050_ReadAll(MPU6050_Data_t *out)
{
    if (out == NULL) return false;

    uint8_t buf[14];
    if (!mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, buf, 14u)) return false;

    int16_t rawAx = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t rawAy = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t rawAz = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t rawT  = (int16_t)((buf[6]  << 8) | buf[7]);
    int16_t rawGx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t rawGy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t rawGz = (int16_t)((buf[12] << 8) | buf[13]);

    out->ax   = (float)rawAx * s_accelScale;
    out->ay   = (float)rawAy * s_accelScale;
    out->az   = (float)rawAz * s_accelScale;
    out->gx   = (float)rawGx * s_gyroScale;
    out->gy   = (float)rawGy * s_gyroScale;
    out->gz   = (float)rawGz * s_gyroScale;
    out->temp = (float)rawT / 340.0f + 36.53f;

    return true;
}

/* ---------- 新增零偏校准功能实现 ---------- */

bool MPU6050_CalibrateGyro(uint16_t samples)
{
    if (samples == 0) return false;

    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    uint16_t validSamples = 0;

    for (uint16_t i = 0; i < samples; i++) {
        MPU6050_Data_t data;
        if (MPU6050_ReadAll(&data)) {
            sumX += data.gx;
            sumY += data.gy;
            sumZ += data.gz;
            validSamples++;
        }
        /* 简单延时，保证采样间隔均匀 */
        volatile uint32_t delay = 1000;  // 约 1ms 延时（32MHz）
        while (delay--) __NOP();
    }

    if (validSamples == 0) return false;

    s_gyroZeroX = sumX / validSamples;
    s_gyroZeroY = sumY / validSamples;
    s_gyroZeroZ = sumZ / validSamples;

    return true;
}

void MPU6050_GetGyroZero(float *x, float *y, float *z)
{
    if (x) *x = s_gyroZeroX;
    if (y) *y = s_gyroZeroY;
    if (z) *z = s_gyroZeroZ;
}

void MPU6050_SetGyroZero(float x, float y, float z)
{
    s_gyroZeroX = x;
    s_gyroZeroY = y;
    s_gyroZeroZ = z;
}

bool MPU6050_ReadAllCalibrated(MPU6050_Data_t *out)
{
    if (!MPU6050_ReadAll(out)) return false;

    /* 减去零偏 */
    out->gx -= s_gyroZeroX;
    out->gy -= s_gyroZeroY;
    out->gz -= s_gyroZeroZ;

    return true;
}
