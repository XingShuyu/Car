/**
 * @file    hwt101.c
 * @brief   HWT101 single-axis WIT I2C driver implementation.
 */

#include "hwt101.h"
#include "ti/driverlib/dl_gpio.h"
#include "ti/driverlib/dl_i2c.h"
#include <stddef.h>

#define HWT101_I2C_TIMEOUT_LOOPS  (1000000u)
#define HWT101_RETRY_COUNT        (3u)
#define HWT101_RECOVERY_PULSES    (9u)
#define HWT101_GPIO_DELAY_LOOPS   (320u)
#define HWT101_UNLOCK_DELAY_LOOPS (32000u)

#define HWT101_RAW_GYRO_TO_DPS    (2000.0f / 32768.0f)
#define HWT101_RAW_ANGLE_TO_DEG   (180.0f / 32768.0f)
#define HWT101_ACCEL_DETECT_LIMIT (1024)

#define HWT101_CLEAR_INTERRUPTS                                           \
    (DL_I2C_INTERRUPT_CONTROLLER_NACK | DL_I2C_INTERRUPT_CONTROLLER_STOP | \
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |                              \
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |                              \
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)

static int16_t make_i16_le(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

static void delay_loop(uint32_t loops)
{
    while (loops-- > 0u) {
        __NOP();
    }
}

static void clear_i2c_state(void)
{
    DL_I2C_resetControllerTransfer(HWT101_I2C_INST);
    DL_I2C_flushControllerTXFIFO(HWT101_I2C_INST);
    DL_I2C_flushControllerRXFIFO(HWT101_I2C_INST);
    DL_I2C_clearInterruptStatus(HWT101_I2C_INST, HWT101_CLEAR_INTERRUPTS);
}

static bool wait_idle(void)
{
    uint32_t timeout = HWT101_I2C_TIMEOUT_LOOPS;

    while ((DL_I2C_getControllerStatus(HWT101_I2C_INST) &
               DL_I2C_CONTROLLER_STATUS_IDLE) == 0u) {
        if (--timeout == 0u) {
            return false;
        }
    }
    return true;
}

static bool wait_stop(void)
{
    uint32_t timeout = HWT101_I2C_TIMEOUT_LOOPS;
    uint32_t status;

    do {
        status = DL_I2C_getRawInterruptStatus(
            HWT101_I2C_INST,
            DL_I2C_INTERRUPT_CONTROLLER_STOP |
                DL_I2C_INTERRUPT_CONTROLLER_NACK |
                DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);
        if ((status & (DL_I2C_INTERRUPT_CONTROLLER_NACK |
                          DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)) !=
            0u) {
            clear_i2c_state();
            return false;
        }
        if (--timeout == 0u) {
            return false;
        }
    } while ((status & DL_I2C_INTERRUPT_CONTROLLER_STOP) == 0u);

    DL_I2C_clearInterruptStatus(HWT101_I2C_INST, HWT101_CLEAR_INTERRUPTS);
    return true;
}

static bool wait_tx_done(void)
{
    uint32_t timeout = HWT101_I2C_TIMEOUT_LOOPS;
    uint32_t status;

    do {
        status = DL_I2C_getRawInterruptStatus(
            HWT101_I2C_INST,
            DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
                DL_I2C_INTERRUPT_CONTROLLER_NACK |
                DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);
        if ((status & (DL_I2C_INTERRUPT_CONTROLLER_NACK |
                          DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)) !=
            0u) {
            clear_i2c_state();
            return false;
        }
        if (--timeout == 0u) {
            return false;
        }
    } while ((status & DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) == 0u);

    DL_I2C_clearInterruptStatus(
        HWT101_I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    return true;
}

static bool read_rx_fifo(uint8_t *buffer, uint8_t length)
{
    for (uint8_t i = 0u; i < length; i++) {
        uint32_t timeout = HWT101_I2C_TIMEOUT_LOOPS;
        while (DL_I2C_isControllerRXFIFOEmpty(HWT101_I2C_INST)) {
            if ((DL_I2C_getRawInterruptStatus(HWT101_I2C_INST,
                     DL_I2C_INTERRUPT_CONTROLLER_NACK |
                         DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST) !=
                    0u) ||
                (--timeout == 0u)) {
                clear_i2c_state();
                return false;
            }
        }
        buffer[i] = DL_I2C_receiveControllerData(HWT101_I2C_INST);
    }
    return true;
}

static void enable_i2c_pullups(void)
{
#if defined(GPIO_I2C_mpu6050_IOMUX_SDA) && \
    defined(GPIO_I2C_mpu6050_IOMUX_SDA_FUNC) && \
    defined(GPIO_I2C_mpu6050_IOMUX_SCL) && \
    defined(GPIO_I2C_mpu6050_IOMUX_SCL_FUNC)
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_I2C_mpu6050_IOMUX_SDA,
        GPIO_I2C_mpu6050_IOMUX_SDA_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_I2C_mpu6050_IOMUX_SCL,
        GPIO_I2C_mpu6050_IOMUX_SCL_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_mpu6050_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_mpu6050_IOMUX_SCL);
#endif
}

static bool recover_bus(void)
{
#if defined(GPIO_I2C_mpu6050_IOMUX_SDA) && defined(GPIO_I2C_mpu6050_IOMUX_SCL)
    clear_i2c_state();

    DL_GPIO_initDigitalOutputFeatures(
        GPIO_I2C_mpu6050_IOMUX_SDA,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_DRIVE_STRENGTH_LOW,
        DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initDigitalOutputFeatures(
        GPIO_I2C_mpu6050_IOMUX_SCL,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_DRIVE_STRENGTH_LOW,
        DL_GPIO_HIZ_DISABLE);

    DL_GPIO_setPins(GPIO_I2C_mpu6050_SDA_PORT, GPIO_I2C_mpu6050_SDA_PIN);
    DL_GPIO_setPins(GPIO_I2C_mpu6050_SCL_PORT, GPIO_I2C_mpu6050_SCL_PIN);
    DL_GPIO_disableOutput(GPIO_I2C_mpu6050_SDA_PORT, GPIO_I2C_mpu6050_SDA_PIN);
    DL_GPIO_disableOutput(GPIO_I2C_mpu6050_SCL_PORT, GPIO_I2C_mpu6050_SCL_PIN);
    delay_loop(HWT101_GPIO_DELAY_LOOPS);

    for (uint8_t i = 0u; i < HWT101_RECOVERY_PULSES; i++) {
        if ((DL_GPIO_readPins(GPIO_I2C_mpu6050_SDA_PORT,
                 GPIO_I2C_mpu6050_SDA_PIN) != 0u) &&
            (DL_GPIO_readPins(GPIO_I2C_mpu6050_SCL_PORT,
                 GPIO_I2C_mpu6050_SCL_PIN) != 0u)) {
            break;
        }

        DL_GPIO_clearPins(
            GPIO_I2C_mpu6050_SCL_PORT, GPIO_I2C_mpu6050_SCL_PIN);
        DL_GPIO_enableOutput(
            GPIO_I2C_mpu6050_SCL_PORT, GPIO_I2C_mpu6050_SCL_PIN);
        delay_loop(HWT101_GPIO_DELAY_LOOPS);
        DL_GPIO_setPins(GPIO_I2C_mpu6050_SCL_PORT, GPIO_I2C_mpu6050_SCL_PIN);
        DL_GPIO_disableOutput(
            GPIO_I2C_mpu6050_SCL_PORT, GPIO_I2C_mpu6050_SCL_PIN);
        delay_loop(HWT101_GPIO_DELAY_LOOPS);
    }

    enable_i2c_pullups();
#endif
    clear_i2c_state();
    return wait_idle();
}

static bool prepare_transfer(void)
{
    if (!wait_idle()) {
        return recover_bus();
    }
    clear_i2c_state();
    return true;
}

static bool read_accel_detect_regs(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t data[6];

    if ((ax == NULL) || (ay == NULL) || (az == NULL)) {
        return false;
    }
    if (!HWT101_ReadBytes(HWT101_REG_AX, data, sizeof(data))) {
        return false;
    }

    *ax = make_i16_le(data[0], data[1]);
    *ay = make_i16_le(data[2], data[3]);
    *az = make_i16_le(data[4], data[5]);
    return true;
}

static bool looks_like_full_attitude_module(void)
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    if (!read_accel_detect_regs(&ax, &ay, &az)) {
        return false;
    }

    return ((ax > HWT101_ACCEL_DETECT_LIMIT) ||
               (ax < -HWT101_ACCEL_DETECT_LIMIT) ||
               (ay > HWT101_ACCEL_DETECT_LIMIT) ||
               (ay < -HWT101_ACCEL_DETECT_LIMIT) ||
               (az > HWT101_ACCEL_DETECT_LIMIT) ||
               (az < -HWT101_ACCEL_DETECT_LIMIT));
}

bool HWT101_ReadBytes(uint8_t startReg, uint8_t *buffer, uint8_t length)
{
    if ((buffer == NULL) || (length == 0u)) {
        return false;
    }

    for (uint8_t retry = 0u; retry < HWT101_RETRY_COUNT; retry++) {
        if (!prepare_transfer() ||
            (DL_I2C_fillControllerTXFIFO(HWT101_I2C_INST, &startReg, 1u) !=
                1u)) {
            recover_bus();
            continue;
        }

        DL_I2C_startControllerTransferAdvanced(
            HWT101_I2C_INST,
            HWT101_ADDR,
            DL_I2C_CONTROLLER_DIRECTION_TX,
            1u,
            DL_I2C_CONTROLLER_START_ENABLE,
            DL_I2C_CONTROLLER_STOP_DISABLE,
            DL_I2C_CONTROLLER_ACK_DISABLE);

        if (!wait_tx_done()) {
            recover_bus();
            continue;
        }

        DL_I2C_flushControllerRXFIFO(HWT101_I2C_INST);
        DL_I2C_clearInterruptStatus(HWT101_I2C_INST, HWT101_CLEAR_INTERRUPTS);
        DL_I2C_startControllerTransferAdvanced(
            HWT101_I2C_INST,
            HWT101_ADDR,
            DL_I2C_CONTROLLER_DIRECTION_RX,
            length,
            DL_I2C_CONTROLLER_START_ENABLE,
            DL_I2C_CONTROLLER_STOP_ENABLE,
            DL_I2C_CONTROLLER_ACK_DISABLE);

        if (read_rx_fifo(buffer, length) && wait_stop()) {
            return true;
        }
        recover_bus();
    }

    return false;
}

bool HWT101_WriteRegister(uint8_t reg, uint16_t value)
{
    uint8_t data[3] = {
        reg,
        (uint8_t)(value & 0xFFu),
        (uint8_t)((value >> 8) & 0xFFu),
    };

    for (uint8_t retry = 0u; retry < HWT101_RETRY_COUNT; retry++) {
        if (prepare_transfer() &&
            (DL_I2C_fillControllerTXFIFO(
                 HWT101_I2C_INST, data, sizeof(data)) == sizeof(data))) {
            DL_I2C_startControllerTransfer(HWT101_I2C_INST, HWT101_ADDR,
                DL_I2C_CONTROLLER_DIRECTION_TX, sizeof(data));
            if (wait_stop()) {
                return true;
            }
        }
        recover_bus();
    }

    return false;
}

bool HWT101_ProbeAddress(uint8_t addr)
{
    if ((addr == 0u) || (addr > 0x7Fu) || !prepare_transfer()) {
        return false;
    }

    DL_I2C_startControllerTransfer(
        HWT101_I2C_INST, addr, DL_I2C_CONTROLLER_DIRECTION_TX, 0u);
    return wait_stop();
}

uint8_t HWT101_ScanFirstAddress(void)
{
    enable_i2c_pullups();

    if (HWT101_ProbeAddress(HWT101_ADDR)) {
        return HWT101_ADDR;
    }
    for (uint8_t addr = 0x08u; addr <= 0x77u; addr++) {
        if ((addr != HWT101_ADDR) && HWT101_ProbeAddress(addr)) {
            return addr;
        }
    }
    return 0u;
}

bool HWT101_ReadRegister(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];

    if (value == NULL) {
        return false;
    }
    if (!HWT101_ReadBytes(reg, data, sizeof(data))) {
        return false;
    }

    *value = (uint16_t)make_i16_le(data[0], data[1]);
    return true;
}

bool HWT101_IsConnected(void)
{
    uint16_t version;
    uint16_t gy;
    uint16_t gz;
    uint16_t yaw;

    if (!HWT101_ProbeAddress(HWT101_ADDR)) {
        return false;
    }
    if (!HWT101_ReadRegister(HWT101_REG_VERSION, &version) ||
        !HWT101_ReadRegister(HWT101_REG_GY, &gy) ||
        !HWT101_ReadRegister(HWT101_REG_GZ, &gz) ||
        !HWT101_ReadRegister(HWT101_REG_YAW, &yaw)) {
        return false;
    }

    if ((version == 0xFFFFu) && (gy == 0xFFFFu) && (gz == 0xFFFFu) &&
        (yaw == 0xFFFFu)) {
        return false;
    }

    return !looks_like_full_attitude_module();
}

bool HWT101_Init(void)
{
    enable_i2c_pullups();
    recover_bus();
    return HWT101_IsConnected();
}

bool HWT101_ZeroYaw(void)
{
    if (!HWT101_WriteRegister(HWT101_REG_KEY, HWT101_KEY_UNLOCK)) {
        return false;
    }

    delay_loop(HWT101_UNLOCK_DELAY_LOOPS);
    if (HWT101_WriteRegister(HWT101_REG_CALIYAW, HWT101_CALIYAW_CLEAR)) {
        delay_loop(HWT101_UNLOCK_DELAY_LOOPS);
        (void)HWT101_WriteRegister(HWT101_REG_CALIYAW, HWT101_CALIYAW_NORMAL);
        return true;
    }

    return HWT101_WriteRegister(HWT101_REG_CALSW, HWT101_CALSW_ZERO_YAW);
}

bool HWT101_ReadGyroRaw(HWT101_GyroRaw_t *out)
{
    uint8_t data[4];

    if ((out == NULL) || !HWT101_ReadBytes(HWT101_REG_GY, data, sizeof(data))) {
        return false;
    }

    out->y = make_i16_le(data[0], data[1]);
    out->z = make_i16_le(data[2], data[3]);
    return true;
}

bool HWT101_ReadYawRaw(int16_t *out)
{
    uint8_t data[2];

    if ((out == NULL) || !HWT101_ReadBytes(HWT101_REG_YAW, data, sizeof(data))) {
        return false;
    }

    *out = make_i16_le(data[0], data[1]);
    return true;
}

bool HWT101_ReadRaw(HWT101_RawData_t *out)
{
    HWT101_GyroRaw_t gyro;
    uint16_t version;
    int16_t yaw;

    if (out == NULL) {
        return false;
    }
    if (!HWT101_ReadGyroRaw(&gyro) || !HWT101_ReadYawRaw(&yaw) ||
        !HWT101_ReadRegister(HWT101_REG_VERSION, &version)) {
        return false;
    }

    out->gy = gyro.y;
    out->gz = gyro.z;
    out->yaw = yaw;
    out->version = (int16_t)version;
    return true;
}

void HWT101_ConvertRaw(const HWT101_RawData_t *raw, IMU_Data_t *out)
{
    if ((raw == NULL) || (out == NULL)) {
        return;
    }

    *out = (IMU_Data_t){0};
    out->gy = (float)raw->gy * HWT101_RAW_GYRO_TO_DPS;
    out->gz = (float)raw->gz * HWT101_RAW_GYRO_TO_DPS;
    out->yaw = (float)raw->yaw * HWT101_RAW_ANGLE_TO_DEG;
}

bool HWT101_ReadAll(IMU_Data_t *out)
{
    HWT101_RawData_t raw;

    if ((out == NULL) || !HWT101_ReadRaw(&raw)) {
        return false;
    }

    HWT101_ConvertRaw(&raw, out);
    return true;
}
