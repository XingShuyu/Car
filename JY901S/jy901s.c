/**
 * @file    jy901s.c
 * @brief   JY901S / WT901 compatible I2C driver implementation.
 */

#include "jy901s.h"
#include "ti/driverlib/dl_gpio.h"
#include "ti/driverlib/dl_i2c.h"
#include <stddef.h>

#define JY901S_I2C_TIMEOUT_LOOPS   (1000000u)
#define JY901S_BUS_RECOVERY_PULSES (9u)
#define JY901S_GPIO_DELAY_LOOPS    (320u)
#define JY901S_TRANSFER_GAP_LOOPS  (160u)
#define JY901S_READ_RETRY_COUNT    (3u)
#define JY901S_WRITE_RETRY_COUNT   (3u)

#define JY901S_RAW_ACCEL_TO_G   (16.0f / 32768.0f)
#define JY901S_RAW_GYRO_TO_DPS  (2000.0f / 32768.0f)
#define JY901S_RAW_ANGLE_TO_DEG (180.0f / 32768.0f)
#define JY901S_RAW_TEMP_TO_DEGC (0.01f)

#define JY901S_ALL_DATA_BYTES \
    (((uint8_t)JY901S_REG_TEMP - (uint8_t)JY901S_REG_AX + 1u) * 2u)
#define JY901S_REG_OFFSET(reg) (((uint8_t)(reg) - (uint8_t)JY901S_REG_AX) * 2u)

#define JY901S_I2C_CLEAR_INTERRUPTS                                      \
    (DL_I2C_INTERRUPT_CONTROLLER_NACK | DL_I2C_INTERRUPT_CONTROLLER_STOP | \
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |                             \
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |                             \
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)

#define JY901S_I2C_END_INTERRUPTS                                      \
    (DL_I2C_INTERRUPT_CONTROLLER_NACK | DL_I2C_INTERRUPT_CONTROLLER_STOP | \
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

static void i2c_clear_controller_state(void)
{
    DL_I2C_resetControllerTransfer(JY901S_I2C_INST);
    DL_I2C_flushControllerTXFIFO(JY901S_I2C_INST);
    DL_I2C_flushControllerRXFIFO(JY901S_I2C_INST);
    DL_I2C_clearInterruptStatus(
        JY901S_I2C_INST, JY901S_I2C_CLEAR_INTERRUPTS);
}

static bool i2c_wait_idle(void)
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

static bool i2c_wait_bus_free(void)
{
    uint32_t timeout = JY901S_I2C_TIMEOUT_LOOPS;

    while ((DL_I2C_getControllerStatus(JY901S_I2C_INST) &
               DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0u) {
        if (--timeout == 0u) {
            return false;
        }
    }
    return i2c_wait_idle();
}

static bool i2c_wait_stop_ok(void)
{
    uint32_t timeout = JY901S_I2C_TIMEOUT_LOOPS;
    uint32_t status;

    do {
        status = DL_I2C_getRawInterruptStatus(
            JY901S_I2C_INST, JY901S_I2C_END_INTERRUPTS);
        if ((status & (DL_I2C_INTERRUPT_CONTROLLER_NACK |
                          DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)) !=
            0u) {
            DL_I2C_clearInterruptStatus(
                JY901S_I2C_INST, JY901S_I2C_CLEAR_INTERRUPTS);
            return false;
        }
        if (--timeout == 0u) {
            return false;
        }
    } while ((status & DL_I2C_INTERRUPT_CONTROLLER_STOP) == 0u);

    DL_I2C_clearInterruptStatus(
        JY901S_I2C_INST, JY901S_I2C_CLEAR_INTERRUPTS);

    return (DL_I2C_getControllerStatus(JY901S_I2C_INST) &
               DL_I2C_CONTROLLER_STATUS_ERROR) == 0u;
}

static bool i2c_wait_tx_done_ok(void)
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
            DL_I2C_clearInterruptStatus(
                JY901S_I2C_INST, JY901S_I2C_CLEAR_INTERRUPTS);
            return false;
        }
        if (--timeout == 0u) {
            return false;
        }
    } while ((status & DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) == 0u);

    DL_I2C_clearInterruptStatus(
        JY901S_I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

    return (DL_I2C_getControllerStatus(JY901S_I2C_INST) &
               DL_I2C_CONTROLLER_STATUS_ERROR) == 0u;
}

static bool i2c_wait_rx_byte(uint8_t *byte)
{
    uint32_t timeout = JY901S_I2C_TIMEOUT_LOOPS;
    uint32_t status;

    while (DL_I2C_isControllerRXFIFOEmpty(JY901S_I2C_INST)) {
        status = DL_I2C_getRawInterruptStatus(
            JY901S_I2C_INST, JY901S_I2C_END_INTERRUPTS);
        if ((status & (DL_I2C_INTERRUPT_CONTROLLER_NACK |
                          DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)) !=
            0u) {
            DL_I2C_clearInterruptStatus(
                JY901S_I2C_INST, JY901S_I2C_CLEAR_INTERRUPTS);
            return false;
        }
        if (--timeout == 0u) {
            return false;
        }
    }

    *byte = DL_I2C_receiveControllerData(JY901S_I2C_INST);
    return true;
}

static void i2c_enable_bus_pullups(void)
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

static void i2c_configure_bus_pins(void)
{
    i2c_enable_bus_pullups();
    i2c_clear_controller_state();
}

static void gpio_release_sda(void)
{
    DL_GPIO_setPins(GPIO_I2C_mpu6050_SDA_PORT, GPIO_I2C_mpu6050_SDA_PIN);
    DL_GPIO_disableOutput(GPIO_I2C_mpu6050_SDA_PORT, GPIO_I2C_mpu6050_SDA_PIN);
}

static void gpio_drive_sda_low(void)
{
    DL_GPIO_clearPins(GPIO_I2C_mpu6050_SDA_PORT, GPIO_I2C_mpu6050_SDA_PIN);
    DL_GPIO_enableOutput(GPIO_I2C_mpu6050_SDA_PORT, GPIO_I2C_mpu6050_SDA_PIN);
}

static void gpio_release_scl(void)
{
    DL_GPIO_setPins(GPIO_I2C_mpu6050_SCL_PORT, GPIO_I2C_mpu6050_SCL_PIN);
    DL_GPIO_disableOutput(GPIO_I2C_mpu6050_SCL_PORT, GPIO_I2C_mpu6050_SCL_PIN);
}

static void gpio_drive_scl_low(void)
{
    DL_GPIO_clearPins(GPIO_I2C_mpu6050_SCL_PORT, GPIO_I2C_mpu6050_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_I2C_mpu6050_SCL_PORT, GPIO_I2C_mpu6050_SCL_PIN);
}

static bool gpio_line_is_high(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0u);
}

static bool i2c_recover_bus(void)
{
#if defined(GPIO_I2C_mpu6050_IOMUX_SDA) && defined(GPIO_I2C_mpu6050_IOMUX_SCL)
    i2c_clear_controller_state();

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

    gpio_release_sda();
    gpio_release_scl();
    delay_loop(JY901S_GPIO_DELAY_LOOPS);

    for (uint8_t i = 0u; i < JY901S_BUS_RECOVERY_PULSES; i++) {
        if (gpio_line_is_high(GPIO_I2C_mpu6050_SDA_PORT,
                GPIO_I2C_mpu6050_SDA_PIN) &&
            gpio_line_is_high(GPIO_I2C_mpu6050_SCL_PORT,
                GPIO_I2C_mpu6050_SCL_PIN)) {
            break;
        }

        gpio_drive_scl_low();
        delay_loop(JY901S_GPIO_DELAY_LOOPS);
        gpio_release_scl();
        delay_loop(JY901S_GPIO_DELAY_LOOPS);
    }

    gpio_drive_sda_low();
    delay_loop(JY901S_GPIO_DELAY_LOOPS);
    gpio_release_scl();
    delay_loop(JY901S_GPIO_DELAY_LOOPS);
    gpio_release_sda();
    delay_loop(JY901S_GPIO_DELAY_LOOPS);

    i2c_configure_bus_pins();

    return gpio_line_is_high(GPIO_I2C_mpu6050_SDA_PORT,
               GPIO_I2C_mpu6050_SDA_PIN) &&
           gpio_line_is_high(GPIO_I2C_mpu6050_SCL_PORT,
               GPIO_I2C_mpu6050_SCL_PIN);
#else
    i2c_configure_bus_pins();
    return false;
#endif
}

static bool i2c_prepare_transfer(void)
{
    if (!i2c_wait_bus_free()) {
        if (!i2c_recover_bus()) {
            return false;
        }
        if (!i2c_wait_bus_free()) {
            return false;
        }
    }

    i2c_clear_controller_state();
    return true;
}

static bool i2c_write_bytes_once(uint8_t addr, const uint8_t *data, uint8_t length)
{
    if ((data == NULL) || (length == 0u)) {
        return false;
    }
    if (!i2c_prepare_transfer()) {
        return false;
    }
    if (DL_I2C_fillControllerTXFIFO(JY901S_I2C_INST, data, length) != length) {
        return false;
    }

    DL_I2C_startControllerTransfer(
        JY901S_I2C_INST,
        addr,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        length);
    delay_loop(JY901S_TRANSFER_GAP_LOOPS);

    return i2c_wait_stop_ok() && i2c_wait_bus_free();
}

static bool i2c_read_bytes_once(uint8_t addr, uint8_t *buffer, uint8_t length)
{
    if ((buffer == NULL) || (length == 0u)) {
        return false;
    }
    if (!i2c_prepare_transfer()) {
        return false;
    }

    DL_I2C_startControllerTransfer(
        JY901S_I2C_INST,
        addr,
        DL_I2C_CONTROLLER_DIRECTION_RX,
        length);
    delay_loop(JY901S_TRANSFER_GAP_LOOPS);

    for (uint8_t i = 0u; i < length; i++) {
        if (!i2c_wait_rx_byte(&buffer[i])) {
            return false;
        }
    }

    return i2c_wait_stop_ok() && i2c_wait_bus_free();
}

static bool i2c_read_register_repeated_once(
    uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length)
{
    if ((buffer == NULL) || (length == 0u)) {
        return false;
    }
    if (!i2c_prepare_transfer()) {
        return false;
    }
    if (DL_I2C_fillControllerTXFIFO(JY901S_I2C_INST, &reg, 1u) != 1u) {
        return false;
    }

    DL_I2C_startControllerTransferAdvanced(
        JY901S_I2C_INST,
        addr,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        1u,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_loop(JY901S_TRANSFER_GAP_LOOPS);

    if (!i2c_wait_tx_done_ok()) {
        return false;
    }

    DL_I2C_flushControllerRXFIFO(JY901S_I2C_INST);
    DL_I2C_clearInterruptStatus(
        JY901S_I2C_INST, JY901S_I2C_CLEAR_INTERRUPTS);

    DL_I2C_startControllerTransferAdvanced(
        JY901S_I2C_INST,
        addr,
        DL_I2C_CONTROLLER_DIRECTION_RX,
        length,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_ENABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_loop(JY901S_TRANSFER_GAP_LOOPS);

    for (uint8_t i = 0u; i < length; i++) {
        if (!i2c_wait_rx_byte(&buffer[i])) {
            return false;
        }
    }

    return i2c_wait_stop_ok() && i2c_wait_bus_free();
}

static bool i2c_probe_once(uint8_t addr)
{
    if ((addr == 0u) || (addr > 0x7Fu)) {
        return false;
    }
    if (!i2c_prepare_transfer()) {
        return false;
    }

    DL_I2C_startControllerTransfer(
        JY901S_I2C_INST,
        addr,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        0u);
    delay_loop(JY901S_TRANSFER_GAP_LOOPS);

    return i2c_wait_stop_ok() && i2c_wait_bus_free();
}

bool JY901S_ReadBytes(uint8_t startReg, uint8_t *buffer, uint8_t length)
{
    if ((buffer == NULL) || (length == 0u)) {
        return false;
    }

    for (uint8_t i = 0u; i < JY901S_READ_RETRY_COUNT; i++) {
        if (i2c_read_register_repeated_once(
                JY901S_ADDR, startReg, buffer, length)) {
            return true;
        }
        i2c_recover_bus();
    }
    return false;
}

bool JY901S_ReadBytesStopMode(uint8_t startReg, uint8_t *buffer, uint8_t length)
{
    if ((buffer == NULL) || (length == 0u)) {
        return false;
    }

    for (uint8_t i = 0u; i < JY901S_READ_RETRY_COUNT; i++) {
        if (i2c_write_bytes_once(JY901S_ADDR, &startReg, 1u)) {
            delay_loop(JY901S_TRANSFER_GAP_LOOPS);
            if (i2c_read_bytes_once(JY901S_ADDR, buffer, length)) {
                return true;
            }
        }
        i2c_recover_bus();
    }
    return false;
}

bool JY901S_WriteRegister(uint8_t reg, uint16_t value)
{
    uint8_t data[3];

    data[0] = reg;
    data[1] = (uint8_t)(value & 0xFFu);
    data[2] = (uint8_t)((value >> 8) & 0xFFu);

    for (uint8_t i = 0u; i < JY901S_WRITE_RETRY_COUNT; i++) {
        if (i2c_write_bytes_once(JY901S_ADDR, data, sizeof(data))) {
            return true;
        }
        i2c_recover_bus();
    }
    return false;
}

bool JY901S_ProbeAddress(uint8_t addr)
{
    if (i2c_probe_once(addr)) {
        return true;
    }

    i2c_recover_bus();
    return i2c_probe_once(addr);
}

uint8_t JY901S_ScanFirstAddress(void)
{
    i2c_configure_bus_pins();

    if (JY901S_ProbeAddress(JY901S_ADDR)) {
        return JY901S_ADDR;
    }

    for (uint8_t addr = 0x08u; addr <= 0x77u; addr++) {
        if (addr == JY901S_ADDR) {
            continue;
        }
        if (JY901S_ProbeAddress(addr)) {
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
    i2c_configure_bus_pins();
    i2c_recover_bus();
    delay_loop(JY901S_TRANSFER_GAP_LOOPS);

    return JY901S_IsConnected();
}

bool JY901S_ReadAccelRaw(JY901S_VectorRaw_t *out)
{
    uint8_t data[6];

    if (out == NULL) {
        return false;
    }
    if (!JY901S_ReadBytes(JY901S_REG_AX, data, sizeof(data))) {
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

    if (out == NULL) {
        return false;
    }
    if (!JY901S_ReadBytes(JY901S_REG_GX, data, sizeof(data))) {
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

    if (out == NULL) {
        return false;
    }
    if (!JY901S_ReadBytes(JY901S_REG_HX, data, sizeof(data))) {
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

    if (out == NULL) {
        return false;
    }
    if (!JY901S_ReadBytes(JY901S_REG_ROLL, data, sizeof(data))) {
        return false;
    }

    out->x = make_i16_le(data[0], data[1]);
    out->y = make_i16_le(data[2], data[3]);
    out->z = make_i16_le(data[4], data[5]);
    return true;
}

static void parse_raw_block(const uint8_t *data, JY901S_RawData_t *out)
{
    uint8_t off;

    off = JY901S_REG_OFFSET(JY901S_REG_AX);
    out->ax = make_i16_le(data[off], data[off + 1u]);
    out->ay = make_i16_le(data[off + 2u], data[off + 3u]);
    out->az = make_i16_le(data[off + 4u], data[off + 5u]);

    off = JY901S_REG_OFFSET(JY901S_REG_GX);
    out->gx = make_i16_le(data[off], data[off + 1u]);
    out->gy = make_i16_le(data[off + 2u], data[off + 3u]);
    out->gz = make_i16_le(data[off + 4u], data[off + 5u]);

    off = JY901S_REG_OFFSET(JY901S_REG_HX);
    out->hx = make_i16_le(data[off], data[off + 1u]);
    out->hy = make_i16_le(data[off + 2u], data[off + 3u]);
    out->hz = make_i16_le(data[off + 4u], data[off + 5u]);

    off = JY901S_REG_OFFSET(JY901S_REG_ROLL);
    out->roll = make_i16_le(data[off], data[off + 1u]);
    out->pitch = make_i16_le(data[off + 2u], data[off + 3u]);
    out->yaw = make_i16_le(data[off + 4u], data[off + 5u]);

    off = JY901S_REG_OFFSET(JY901S_REG_TEMP);
    out->temp = make_i16_le(data[off], data[off + 1u]);
}

static bool read_raw_segmented(JY901S_RawData_t *out)
{
    uint8_t accel[6];
    uint8_t gyro[6];
    uint8_t mag[6];
    uint8_t angle[6];
    uint8_t temp[2];

    if (!JY901S_ReadBytes(JY901S_REG_AX, accel, sizeof(accel)) ||
        !JY901S_ReadBytes(JY901S_REG_GX, gyro, sizeof(gyro)) ||
        !JY901S_ReadBytes(JY901S_REG_HX, mag, sizeof(mag)) ||
        !JY901S_ReadBytes(JY901S_REG_ROLL, angle, sizeof(angle)) ||
        !JY901S_ReadBytes(JY901S_REG_TEMP, temp, sizeof(temp))) {
        return false;
    }

    out->ax = make_i16_le(accel[0], accel[1]);
    out->ay = make_i16_le(accel[2], accel[3]);
    out->az = make_i16_le(accel[4], accel[5]);
    out->gx = make_i16_le(gyro[0], gyro[1]);
    out->gy = make_i16_le(gyro[2], gyro[3]);
    out->gz = make_i16_le(gyro[4], gyro[5]);
    out->hx = make_i16_le(mag[0], mag[1]);
    out->hy = make_i16_le(mag[2], mag[3]);
    out->hz = make_i16_le(mag[4], mag[5]);
    out->roll = make_i16_le(angle[0], angle[1]);
    out->pitch = make_i16_le(angle[2], angle[3]);
    out->yaw = make_i16_le(angle[4], angle[5]);
    out->temp = make_i16_le(temp[0], temp[1]);
    return true;
}

bool JY901S_ReadRaw(JY901S_RawData_t *out)
{
    uint8_t data[JY901S_ALL_DATA_BYTES];

    if (out == NULL) {
        return false;
    }

    if (JY901S_ReadBytes(JY901S_REG_AX, data, sizeof(data))) {
        parse_raw_block(data, out);
        return true;
    }

    return read_raw_segmented(out);
}

void JY901S_ConvertRaw(const JY901S_RawData_t *raw, JY901S_Data_t *out)
{
    if ((raw == NULL) || (out == NULL)) {
        return;
    }

    out->ax = (float)raw->ax * JY901S_RAW_ACCEL_TO_G;
    out->ay = (float)raw->ay * JY901S_RAW_ACCEL_TO_G;
    out->az = (float)raw->az * JY901S_RAW_ACCEL_TO_G;

    out->gx = (float)raw->gx * JY901S_RAW_GYRO_TO_DPS;
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

    if (out == NULL) {
        return false;
    }
    if (!JY901S_ReadRaw(&raw)) {
        return false;
    }

    JY901S_ConvertRaw(&raw, out);
    return true;
}
