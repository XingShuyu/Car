/**
 * @file    mpu6050.c
 * @brief   MPU6050 driver implementation for TI MSPM0G3507.
 */

#include "mpu6050.h"
#include "ti/driverlib/dl_i2c.h"
#include "BasicMicroLib/delay.h"
#include <math.h>
#include <stddef.h>

#define MPU6050_I2C_TIMEOUT_LOOPS      (1000000u)
#define MPU6050_RESET_DELAY_LOOPS      (100000u)
#define MPU6050_CALIB_DELAY_LOOPS      (1000u)

typedef struct {
    float x;
    float y;
    float z;
} MPU6050_AccelOffset_t;

static float s_accelScale = 1.0f / 16384.0f;
static float s_gyroScale = 1.0f / 131.0f;

static MPU6050_GyroOffset_t s_gyroOffset = {0.0f, 0.0f, 0.0f};
static MPU6050_AccelOffset_t s_accelOffset = {0.0f, 0.0f, 0.0f};
static float s_filterAlpha = 1.0f;
static float s_gyroDeadzoneDps = 0.05f;
static float s_accelDeadzoneG = 0.0f;
static JY901S_Data_t s_prevFiltered = {0};
static bool s_filterReady = false;

static void delay_loop(uint32_t loops)
{
    while (loops-- > 0u) {
        __NOP();
    }
}

static int16_t make_i16(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

static float accel_scale_from_fs(MPU6050_AccelFS_t fs)
{
    switch (fs) {
    case MPU6050_ACCEL_FS_2G:
        return 1.0f / 16384.0f;
    case MPU6050_ACCEL_FS_4G:
        return 1.0f / 8192.0f;
    case MPU6050_ACCEL_FS_8G:
        return 1.0f / 4096.0f;
    case MPU6050_ACCEL_FS_16G:
        return 1.0f / 2048.0f;
    default:
        return 0.0f;
    }
}

static float gyro_scale_from_fs(MPU6050_GyroFS_t fs)
{
    switch (fs) {
    case MPU6050_GYRO_FS_250DPS:
        return 1.0f / 131.0f;
    case MPU6050_GYRO_FS_500DPS:
        return 1.0f / 65.5f;
    case MPU6050_GYRO_FS_1000DPS:
        return 1.0f / 32.8f;
    case MPU6050_GYRO_FS_2000DPS:
        return 1.0f / 16.4f;
    default:
        return 0.0f;
    }
}

static void reset_filter_state(void)
{
    s_prevFiltered = (JY901S_Data_t){0};
    s_filterReady = false;
}

static bool i2c_wait_idle(void)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT_LOOPS;
    while ((DL_I2C_getControllerStatus(MPU6050_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0u) {
        if (--timeout == 0u) {
            return false;
        }
    }
    return true;
}

static bool i2c_wait_stop_or_nack(void)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT_LOOPS;
    uint32_t status;

    do {
        status = DL_I2C_getRawInterruptStatus(
            MPU6050_I2C_INST,
            DL_I2C_INTERRUPT_CONTROLLER_STOP |
                DL_I2C_INTERRUPT_CONTROLLER_NACK);
        if (--timeout == 0u) {
            return false;
        }
    } while (status == 0u);

    DL_I2C_clearInterruptStatus(
        MPU6050_I2C_INST,
        DL_I2C_INTERRUPT_CONTROLLER_STOP |
            DL_I2C_INTERRUPT_CONTROLLER_NACK);

    return (status & DL_I2C_INTERRUPT_CONTROLLER_NACK) == 0u;
}

static bool write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};

    if (!i2c_wait_idle()) {
        return false;
    }

    DL_I2C_clearInterruptStatus(
        MPU6050_I2C_INST,
        DL_I2C_INTERRUPT_CONTROLLER_NACK |
            DL_I2C_INTERRUPT_CONTROLLER_STOP);
    DL_I2C_flushControllerTXFIFO(MPU6050_I2C_INST);
    DL_I2C_fillControllerTXFIFO(MPU6050_I2C_INST, data, sizeof(data));

    DL_I2C_startControllerTransfer(
        MPU6050_I2C_INST,
        MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        sizeof(data));

    return i2c_wait_stop_or_nack();
}

static bool read_regs(uint8_t startReg, uint8_t *buffer, uint8_t length)
{
    if ((buffer == NULL) || (length == 0u)) {
        return false;
    }

    if (!i2c_wait_idle()) {
        return false;
    }

    DL_I2C_clearInterruptStatus(
        MPU6050_I2C_INST,
        DL_I2C_INTERRUPT_CONTROLLER_NACK |
            DL_I2C_INTERRUPT_CONTROLLER_STOP |
            DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
    DL_I2C_flushControllerTXFIFO(MPU6050_I2C_INST);
    DL_I2C_fillControllerTXFIFO(MPU6050_I2C_INST, &startReg, 1u);

    DL_I2C_startControllerTransfer(
        MPU6050_I2C_INST,
        MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        1u);

    if (!i2c_wait_stop_or_nack() || !i2c_wait_idle()) {
        return false;
    }

    DL_I2C_clearInterruptStatus(
        MPU6050_I2C_INST,
        DL_I2C_INTERRUPT_CONTROLLER_NACK |
            DL_I2C_INTERRUPT_CONTROLLER_STOP |
            DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
    DL_I2C_flushControllerRXFIFO(MPU6050_I2C_INST);

    DL_I2C_startControllerTransfer(
        MPU6050_I2C_INST,
        MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX,
        length);

    for (uint8_t i = 0u; i < length; i++) {
        uint32_t timeout = MPU6050_I2C_TIMEOUT_LOOPS;
        while (DL_I2C_isControllerRXFIFOEmpty(MPU6050_I2C_INST)) {
            if (--timeout == 0u) {
                return false;
            }
        }
        buffer[i] = DL_I2C_receiveControllerData(MPU6050_I2C_INST);
    }

    return i2c_wait_stop_or_nack();
}

static void apply_gyro_offset(JY901S_Data_t *data)
{
    data->gx -= s_gyroOffset.x;
    data->gy -= s_gyroOffset.y;
    data->gz -= s_gyroOffset.z;
}

static void apply_accel_offset(JY901S_Data_t *data)
{
    data->ax -= s_accelOffset.x;
    data->ay -= s_accelOffset.y;
    data->az -= s_accelOffset.z;
}

static float apply_deadzone(float value, float threshold)
{
    if ((threshold > 0.0f) && (fabsf(value) < threshold)) {
        return 0.0f;
    }
    return value;
}

static void apply_filter(JY901S_Data_t *data)
{
    if (s_filterAlpha >= 1.0f || !s_filterReady) {
        s_prevFiltered = *data;
        s_filterReady = true;
        return;
    }

    data->ax = s_filterAlpha * data->ax +
               (1.0f - s_filterAlpha) * s_prevFiltered.ax;
    data->ay = s_filterAlpha * data->ay +
               (1.0f - s_filterAlpha) * s_prevFiltered.ay;
    data->az = s_filterAlpha * data->az +
               (1.0f - s_filterAlpha) * s_prevFiltered.az;
    data->gx = s_filterAlpha * data->gx +
               (1.0f - s_filterAlpha) * s_prevFiltered.gx;
    data->gy = s_filterAlpha * data->gy +
               (1.0f - s_filterAlpha) * s_prevFiltered.gy;
    data->gz = s_filterAlpha * data->gz +
               (1.0f - s_filterAlpha) * s_prevFiltered.gz;

    s_prevFiltered = *data;
}

bool MPU6050_WhoAmI(uint8_t *id)
{
    return read_regs(MPU6050_REG_WHO_AM_I, id, 1u);
}

bool MPU6050_Reset(void)
{
    if (!write_reg(MPU6050_REG_PWR_MGMT_1, 0x80u)) {
        return false;
    }
    delay_loop(MPU6050_RESET_DELAY_LOOPS);
    reset_filter_state();
    return true;
}

bool MPU6050_SetGyroFS(MPU6050_GyroFS_t fs)
{
    float scale = gyro_scale_from_fs(fs);
    if (scale <= 0.0f) {
        return false;
    }

    if (!write_reg(MPU6050_REG_GYRO_CONFIG, (uint8_t)fs)) {
        return false;
    }

    s_gyroScale = scale;
    reset_filter_state();
    return true;
}

bool MPU6050_SetAccelFS(MPU6050_AccelFS_t fs)
{
    float scale = accel_scale_from_fs(fs);
    if (scale <= 0.0f) {
        return false;
    }

    if (!write_reg(MPU6050_REG_ACCEL_CONFIG, (uint8_t)fs)) {
        return false;
    }

    s_accelScale = scale;
    reset_filter_state();
    return true;
}

bool MPU6050_Init(void)
{
    MPU6050_Config_t defaultCfg = {
        .gyroFS = MPU6050_GYRO_FS_250DPS,
        .accelFS = MPU6050_ACCEL_FS_2G,
        .dlpfConfig = 0x03u,
        .sampleRateDiv = 0x09u,
    };
    return MPU6050_InitWithConfig(&defaultCfg);
}

bool MPU6050_InitWithConfig(const MPU6050_Config_t *cfg)
{
    uint8_t whoami = 0u;

    if (cfg == NULL) {
        return false;
    }

    if ((gyro_scale_from_fs(cfg->gyroFS) <= 0.0f) ||
        (accel_scale_from_fs(cfg->accelFS) <= 0.0f) ||
        (cfg->dlpfConfig > 0x06u)) {
        return false;
    }

    if (!MPU6050_WhoAmI(&whoami) || (whoami != MPU6050_WHO_AM_I_VAL)) {
        return false;
    }

    if (!MPU6050_Reset()) {
        return false;
    }

    if (!write_reg(MPU6050_REG_PWR_MGMT_1, 0x01u)) {
        return false;
    }
    if (!write_reg(MPU6050_REG_PWR_MGMT_2, 0x00u)) {
        return false;
    }
    if (!write_reg(MPU6050_REG_CONFIG, cfg->dlpfConfig)) {
        return false;
    }
    if (!write_reg(MPU6050_REG_SMPLRT_DIV, cfg->sampleRateDiv)) {
        return false;
    }
    if (!MPU6050_SetGyroFS(cfg->gyroFS)) {
        return false;
    }
    if (!MPU6050_SetAccelFS(cfg->accelFS)) {
        return false;
    }

    MPU6050_SetGyroZero(0.0f, 0.0f, 0.0f);
    s_accelOffset = (MPU6050_AccelOffset_t){0};
    reset_filter_state();
    return true;
}

bool MPU6050_ReadAccelRaw(MPU6050_RawData_t *out)
{
    uint8_t data[6];

    if (out == NULL) {
        return false;
    }
    if (!read_regs(MPU6050_REG_ACCEL_XOUT_H, data, sizeof(data))) {
        return false;
    }

    out->x = make_i16(data[0], data[1]);
    out->y = make_i16(data[2], data[3]);
    out->z = make_i16(data[4], data[5]);
    return true;
}

bool MPU6050_ReadGyroRaw(MPU6050_RawData_t *out)
{
    uint8_t data[6];

    if (out == NULL) {
        return false;
    }
    if (!read_regs(MPU6050_REG_GYRO_XOUT_H, data, sizeof(data))) {
        return false;
    }

    out->x = make_i16(data[0], data[1]);
    out->y = make_i16(data[2], data[3]);
    out->z = make_i16(data[4], data[5]);
    return true;
}

bool MPU6050_ReadTempRaw(int16_t *out)
{
    uint8_t data[2];

    if (out == NULL) {
        return false;
    }
    if (!read_regs(MPU6050_REG_TEMP_OUT_H, data, sizeof(data))) {
        return false;
    }

    *out = make_i16(data[0], data[1]);
    return true;
}

bool MPU6050_ReadAll(JY901S_Data_t *out)
{
    uint8_t data[14];
    int16_t rawAx;
    int16_t rawAy;
    int16_t rawAz;
    int16_t rawTemp;
    int16_t rawGx;
    int16_t rawGy;
    int16_t rawGz;

    if (out == NULL) {
        return false;
    }
    if (!read_regs(MPU6050_REG_ACCEL_XOUT_H, data, sizeof(data))) {
        return false;
    }

    rawAx = make_i16(data[0], data[1]);
    rawAy = make_i16(data[2], data[3]);
    rawAz = make_i16(data[4], data[5]);
    rawTemp = make_i16(data[6], data[7]);
    rawGx = make_i16(data[8], data[9]);
    rawGy = make_i16(data[10], data[11]);
    rawGz = make_i16(data[12], data[13]);

    out->ax = (float)rawAx * s_accelScale;
    out->ay = (float)rawAy * s_accelScale;
    out->az = (float)rawAz * s_accelScale;
    out->gx = (float)rawGx * s_gyroScale;
    out->gy = (float)rawGy * s_gyroScale;
    out->gz = (float)rawGz * s_gyroScale;
    out->hx = 0.0f;
    out->hy = 0.0f;
    out->hz = 0.0f;
    out->roll = 0.0f;
    out->pitch = 0.0f;
    out->yaw = 0.0f;
    out->temp = (float)rawTemp / 340.0f + 36.53f;
    return true;
}

bool MPU6050_CalibrateGyro(uint16_t samples)
{
    MPU6050_GyroOffset_t oldOffset = s_gyroOffset;
    MPU6050_AccelOffset_t oldAccelOffset = s_accelOffset;
    float gyroSumX = 0.0f;
    float gyroSumY = 0.0f;
    float gyroSumZ = 0.0f;
    float accelSumX = 0.0f;
    float accelSumY = 0.0f;
    float accelSumZ = 0.0f;
    uint16_t validGyroSamples = 0u;
    uint16_t validAccelSamples = 0u;

    if (samples == 0u) {
        return false;
    }

    MPU6050_SetGyroZero(0.0f, 0.0f, 0.0f);
    s_accelOffset = (MPU6050_AccelOffset_t){0};

    for (uint16_t i = 0u; i < samples; i++) {
        JY901S_Data_t data;
        if (MPU6050_ReadAll(&data)&&!(data.gx==0.0&&data.gy==0.0&&data.gz==0.0)) {
            gyroSumX += data.gx;
            gyroSumY += data.gy;
            gyroSumZ += data.gz;
            validGyroSamples++;
        }
        delay_ms(25);
        // delay_loop(MPU6050_CALIB_DELAY_LOOPS);
    }

    for (uint16_t i = 0u; i < samples; i++) {
        JY901S_Data_t data;
        if (MPU6050_ReadAll(&data)&&!(data.ax==0.0&&data.ay==0.0&&data.az==0.0)) {
            accelSumX += data.ax;
            accelSumY += data.ay;
            accelSumZ += data.az;
            validAccelSamples++;
        }
        delay_ms(25);
        // delay_loop(MPU6050_CALIB_DELAY_LOOPS);
    }

    if ((validGyroSamples == 0u) || (validAccelSamples == 0u)) {
        s_gyroOffset = oldOffset;
        s_accelOffset = oldAccelOffset;
        return false;
    }

    s_gyroOffset.x = gyroSumX / (float)validGyroSamples;
    s_gyroOffset.y = gyroSumY / (float)validGyroSamples;
    s_gyroOffset.z = gyroSumZ / (float)validGyroSamples;
    s_accelOffset.x = accelSumX / (float)validAccelSamples;
    s_accelOffset.y = accelSumY / (float)validAccelSamples;
    s_accelOffset.z = accelSumZ / (float)validAccelSamples;
    reset_filter_state();
    return true;
}

void MPU6050_GetGyroZero(float *x, float *y, float *z)
{
    if (x != NULL) {
        *x = s_gyroOffset.x;
    }
    if (y != NULL) {
        *y = s_gyroOffset.y;
    }
    if (z != NULL) {
        *z = s_gyroOffset.z;
    }
}

void MPU6050_SetGyroZero(float x, float y, float z)
{
    s_gyroOffset.x = x;
    s_gyroOffset.y = y;
    s_gyroOffset.z = z;
    reset_filter_state();
}

void MPU6050_SetFilter(float alpha, float gyroDeadzoneDps, float accelDeadzoneG)
{
    if (alpha < 0.0f) {
        alpha = 0.0f;
    } else if (alpha > 1.0f) {
        alpha = 1.0f;
    }

    s_filterAlpha = alpha;
    s_gyroDeadzoneDps = (gyroDeadzoneDps > 0.0f) ? gyroDeadzoneDps : 0.0f;
    s_accelDeadzoneG = (accelDeadzoneG > 0.0f) ? accelDeadzoneG : 0.0f;
    reset_filter_state();
}

void MPU6050_SetFilterParam(float alpha, float deadzone)
{
    MPU6050_SetFilter(alpha, deadzone, 0.0f);
}

bool MPU6050_ReadAllCalibrated(JY901S_Data_t *out)
{
    if (out == NULL) {
        return false;
    }
    if (!MPU6050_ReadAll(out)) {
        return false;
    }

    apply_gyro_offset(out);
    apply_accel_offset(out);
    out->gx = apply_deadzone(out->gx, s_gyroDeadzoneDps);
    out->gy = apply_deadzone(out->gy, s_gyroDeadzoneDps);
    out->gz = apply_deadzone(out->gz, s_gyroDeadzoneDps);
    out->ax = apply_deadzone(out->ax, s_accelDeadzoneG);
    out->ay = apply_deadzone(out->ay, s_accelDeadzoneG);
    out->az = apply_deadzone(out->az, s_accelDeadzoneG);
    apply_filter(out);
    return true;
}
