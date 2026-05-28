/**
 * @file    jy901s.c
 * @brief   JY901S / WT901 compatible I2C driver implementation.
 */

#include "jy901s.h"
#include "ti/driverlib/dl_gpio.h"
#include "ti/driverlib/dl_i2c.h"
#include <stddef.h>

#define JY901S_I2C_TIMEOUT_LOOPS (1000000u)
#define JY901S_RETRY_COUNT       (3u)
#define JY901S_RECOVERY_PULSES   (9u)
#define JY901S_GPIO_DELAY_LOOPS  (320u)
#define JY901S_UNLOCK_DELAY_LOOPS (32000u)

#define JY901S_RAW_ACCEL_TO_G   (16.0f / 32768.0f)
#define JY901S_RAW_GYRO_TO_DPS  (2000.0f / 32768.0f)
#define JY901S_RAW_ANGLE_TO_DEG (180.0f / 32768.0f)
#define JY901S_RAW_TEMP_TO_DEGC (0.01f)

#define JY901S_CLEAR_INTERRUPTS                                            \
    (DL_I2C_INTERRUPT_CONTROLLER_NACK | DL_I2C_INTERRUPT_CONTROLLER_STOP |  \
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |                               \
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |                               \
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
    DL_I2C_resetControllerTransfer(JY901S_I2C_INST);
    DL_I2C_flushControllerTXFIFO(JY901S_I2C_INST);
    DL_I2C_flushControllerRXFIFO(JY901S_I2C_INST);
    DL_I2C_clearInterruptStatus(JY901S_I2C_INST, JY901S_CLEAR_INTERRUPTS);
}

static bool wait_idle(void)
{
    uint32_t timeout = JY901S_I2C_TIMEOUT_LOOPS;

    while ((DL_I2C_getControllerStatus(JY901S_I2C_INST) &
               DL_I2C_CONTROLLER_STATUS_IDLE) == 0u) {
        if (--timeout == 0u) {
            return false;
        }
    }
    return true;
}

static bool wait_stop(void)
{
    uint32_t timeout = JY901S_I2C_TIMEOUT_LOOPS;
    uint32_t status;

    do {
        status = DL_I2C_getRawInterruptStatus(
            JY901S_I2C_INST,
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

    DL_I2C_clearInterruptStatus(JY901S_I2C_INST, JY901S_CLEAR_INTERRUPTS);
    return true;
}

static bool wait_tx_done(void)
{
    uint32_t timeout = JY901S_I2C_TIMEOUT_LOOPS;
    uint32_t status;

    do {
        status = DL_I2C_getRawInterruptStatus(
            JY901S_I2C_INST,
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
        JY901S_I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    return true;
}

static bool read_rx_fifo(uint8_t *buffer, uint8_t length)
{
    for (uint8_t i = 0u; i < length; i++) {
        uint32_t timeout = JY901S_I2C_TIMEOUT_LOOPS;
        while (DL_I2C_isControllerRXFIFOEmpty(JY901S_I2C_INST)) {
            if ((DL_I2C_getRawInterruptStatus(JY901S_I2C_INST,
                     DL_I2C_INTERRUPT_CONTROLLER_NACK |
                         DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST) !=
                    0u) ||
                (--timeout == 0u)) {
                clear_i2c_state();
                return false;
            }
        }
        buffer[i] = DL_I2C_receiveControllerData(JY901S_I2C_INST);
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
    delay_loop(JY901S_GPIO_DELAY_LOOPS);

    for (uint8_t i = 0u; i < JY901S_RECOVERY_PULSES; i++) {
        if ((DL_GPIO_readPins(GPIO_I2C_mpu6050_SDA_PORT,
                 GPIO_I2C_mpu6050_SDA_PIN) != 0u) &&
            (DL_GPIO_readPins(GPIO_I2C_mpu6050_SCL_PORT,
                 GPIO_I2C_mpu6050_SCL_PIN) != 0u)) {
            break;
        }

        DL_GPIO_clearPins(GPIO_I2C_mpu6050_SCL_PORT,
            GPIO_I2C_mpu6050_SCL_PIN);
        DL_GPIO_enableOutput(GPIO_I2C_mpu6050_SCL_PORT,
            GPIO_I2C_mpu6050_SCL_PIN);
        delay_loop(JY901S_GPIO_DELAY_LOOPS);
        DL_GPIO_setPins(GPIO_I2C_mpu6050_SCL_PORT, GPIO_I2C_mpu6050_SCL_PIN);
        DL_GPIO_disableOutput(GPIO_I2C_mpu6050_SCL_PORT,
            GPIO_I2C_mpu6050_SCL_PIN);
        delay_loop(JY901S_GPIO_DELAY_LOOPS);
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

bool JY901S_ReadBytes(uint8_t startReg, uint8_t *buffer, uint8_t length)
{
    if ((buffer == NULL) || (length == 0u)) {
        return false;
    }

    for (uint8_t retry = 0u; retry < JY901S_RETRY_COUNT; retry++) {
        if (!prepare_transfer() ||
            (DL_I2C_fillControllerTXFIFO(JY901S_I2C_INST, &startReg, 1u) !=
                1u)) {
            recover_bus();
            continue;
        }

        DL_I2C_startControllerTransferAdvanced(
            JY901S_I2C_INST,
            JY901S_ADDR,
            DL_I2C_CONTROLLER_DIRECTION_TX,
            1u,
            DL_I2C_CONTROLLER_START_ENABLE,
            DL_I2C_CONTROLLER_STOP_DISABLE,
            DL_I2C_CONTROLLER_ACK_DISABLE);

        if (!wait_tx_done()) {
            recover_bus();
            continue;
        }

        DL_I2C_flushControllerRXFIFO(JY901S_I2C_INST);
        DL_I2C_clearInterruptStatus(JY901S_I2C_INST, JY901S_CLEAR_INTERRUPTS);
        DL_I2C_startControllerTransferAdvanced(
            JY901S_I2C_INST,
            JY901S_ADDR,
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

bool JY901S_WriteRegister(uint8_t reg, uint16_t value)
{
    uint8_t data[3] = {
        reg,
        (uint8_t)(value & 0xFFu),
        (uint8_t)((value >> 8) & 0xFFu),
    };

    for (uint8_t retry = 0u; retry < JY901S_RETRY_COUNT; retry++) {
        if (prepare_transfer() &&
            (DL_I2C_fillControllerTXFIFO(JY901S_I2C_INST, data,
                 sizeof(data)) == sizeof(data))) {
            DL_I2C_startControllerTransfer(JY901S_I2C_INST, JY901S_ADDR,
                DL_I2C_CONTROLLER_DIRECTION_TX, sizeof(data));
            if (wait_stop()) {
                return true;
            }
        }
        recover_bus();
    }

    return false;
}

bool JY901S_ProbeAddress(uint8_t addr)
{
    if ((addr == 0u) || (addr > 0x7Fu) || !prepare_transfer()) {
        return false;
    }

    DL_I2C_startControllerTransfer(
        JY901S_I2C_INST, addr, DL_I2C_CONTROLLER_DIRECTION_TX, 0u);
    return wait_stop();
}

uint8_t JY901S_ScanFirstAddress(void)
{
    enable_i2c_pullups();

    if (JY901S_ProbeAddress(JY901S_ADDR)) {
        return JY901S_ADDR;
    }
    for (uint8_t addr = 0x08u; addr <= 0x77u; addr++) {
        if ((addr != JY901S_ADDR) && JY901S_ProbeAddress(addr)) {
            return addr;
        }
    }
    return 0u;
}

bool JY901S_ReadRegister(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];

    if (value == NULL) {
        return false;
    }
    if (!JY901S_ReadBytes(reg, data, sizeof(data))) {
        return false;
    }

    *value = (uint16_t)make_i16_le(data[0], data[1]);
    return true;
}

bool JY901S_IsConnected(void)
{
    uint8_t data[2];

    return JY901S_ProbeAddress(JY901S_ADDR) &&
           JY901S_ReadBytes(JY901S_REG_AX, data, sizeof(data));
}

bool JY901S_Init(void)
{
    enable_i2c_pullups();
    recover_bus();
    return JY901S_IsConnected();
}

bool JY901S_ZeroYaw(void)
{
    if (!JY901S_WriteRegister(JY901S_REG_KEY, JY901S_KEY_UNLOCK)) {
        return false;
    }

    delay_loop(JY901S_UNLOCK_DELAY_LOOPS);
    return JY901S_WriteRegister(JY901S_REG_CALSW, JY901S_CALSW_ZERO_YAW);
}

bool JY901S_ReadAccelRaw(JY901S_VectorRaw_t *out)
{
    uint8_t data[6];

    if ((out == NULL) || !JY901S_ReadBytes(JY901S_REG_AX, data, sizeof(data))) {
        return false;
    }

    out->x = make_i16_le(data[0], data[1]);
    out->y = make_i16_le(data[2], data[3]);
    out->z = make_i16_le(data[4], data[5]);
    return true;
}

bool JY901S_ReadGyroRaw(JY901S_VectorRaw_t *out)
{
    uint8_t data[6];

    if ((out == NULL) || !JY901S_ReadBytes(JY901S_REG_GX, data, sizeof(data))) {
        return false;
    }

    out->x = make_i16_le(data[0], data[1]);
    out->y = make_i16_le(data[2], data[3]);
    out->z = make_i16_le(data[4], data[5]);
    return true;
}

bool JY901S_ReadMagRaw(JY901S_VectorRaw_t *out)
{
    uint8_t data[6];

    if ((out == NULL) || !JY901S_ReadBytes(JY901S_REG_HX, data, sizeof(data))) {
        return false;
    }

    out->x = make_i16_le(data[0], data[1]);
    out->y = make_i16_le(data[2], data[3]);
    out->z = make_i16_le(data[4], data[5]);
    return true;
}

bool JY901S_ReadAngleRaw(JY901S_VectorRaw_t *out)
{
    uint8_t data[6];

    if ((out == NULL) ||
        !JY901S_ReadBytes(JY901S_REG_ROLL, data, sizeof(data))) {
        return false;
    }

    out->x = make_i16_le(data[0], data[1]);
    out->y = make_i16_le(data[2], data[3]);
    out->z = make_i16_le(data[4], data[5]);
    return true;
}

bool JY901S_ReadRaw(JY901S_RawData_t *out)
{
    uint8_t data[26];

    if ((out == NULL) ||
        !JY901S_ReadBytes(JY901S_REG_AX, data, sizeof(data))) {
        return false;
    }

    out->ax = make_i16_le(data[0], data[1]);
    out->ay = make_i16_le(data[2], data[3]);
    out->az = make_i16_le(data[4], data[5]);
    out->gx = make_i16_le(data[6], data[7]);
    out->gy = make_i16_le(data[8], data[9]);
    out->gz = make_i16_le(data[10], data[11]);
    out->hx = make_i16_le(data[12], data[13]);
    out->hy = make_i16_le(data[14], data[15]);
    out->hz = make_i16_le(data[16], data[17]);
    out->roll = make_i16_le(data[18], data[19]);
    out->pitch = make_i16_le(data[20], data[21]);
    out->yaw = make_i16_le(data[22], data[23]);
    out->temp = make_i16_le(data[24], data[25]);
    return true;
}

void JY901S_ConvertRaw(const JY901S_RawData_t *raw, JY901S_Data_t *out)
{
    if ((raw == NULL) || (out == NULL)) {
        return;
    }

    out->ax = (float)raw->ax * JY901S_RAW_ACCEL_TO_G;
    out->ay = (float)raw->ay * JY901S_RAW_ACCEL_TO_G;
    out->az = (float)raw->az * JY901S_RAW_ACCEL_TO_G;
    out->gx = -((float)raw->gx * JY901S_RAW_GYRO_TO_DPS);
    out->gy = (float)raw->gy * JY901S_RAW_GYRO_TO_DPS;
    out->gz = (float)raw->gz * JY901S_RAW_GYRO_TO_DPS;
    out->hx = (float)raw->hx;
    out->hy = (float)raw->hy;
    out->hz = (float)raw->hz;
    out->roll = (float)raw->roll * JY901S_RAW_ANGLE_TO_DEG;
    out->pitch = (float)raw->pitch * JY901S_RAW_ANGLE_TO_DEG;
    out->yaw = (float)raw->yaw * JY901S_RAW_ANGLE_TO_DEG;
    out->temp = (float)raw->temp * JY901S_RAW_TEMP_TO_DEGC;
}

bool JY901S_ReadAll(JY901S_Data_t *out)
{
    JY901S_RawData_t raw;

    if ((out == NULL) || !JY901S_ReadRaw(&raw)) {
        return false;
    }

    JY901S_ConvertRaw(&raw, out);
    return true;
}
