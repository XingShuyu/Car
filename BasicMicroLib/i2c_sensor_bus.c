/**
 * @file i2c_sensor_bus.c
 * @brief Polling transfer implementation for the PA28/PA31 I2C0 sensor bus.
 */

#include "BasicMicroLib/i2c_sensor_bus.h"
#include "ti_msp_dl_config.h"
#include <stddef.h>

#define I2C_SENSOR_BUS_INST         I2C_mpu6050_INST
#define I2C_SENSOR_BUS_TIMEOUT      (1000000u)
#define I2C_SENSOR_BUS_TX_FIFO_SIZE (8u)

#define I2C_SENSOR_BUS_ERROR_FLAGS                                      \
    (DL_I2C_INTERRUPT_CONTROLLER_NACK |                                  \
     DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)

#define I2C_SENSOR_BUS_CLEAR_FLAGS                                      \
    (I2C_SENSOR_BUS_ERROR_FLAGS | DL_I2C_INTERRUPT_CONTROLLER_STOP |    \
     DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |                               \
     DL_I2C_INTERRUPT_CONTROLLER_TX_DONE)

static void clear_transfer_state(void)
{
    DL_I2C_resetControllerTransfer(I2C_SENSOR_BUS_INST);
    DL_I2C_flushControllerTXFIFO(I2C_SENSOR_BUS_INST);
    DL_I2C_flushControllerRXFIFO(I2C_SENSOR_BUS_INST);
    DL_I2C_clearInterruptStatus(I2C_SENSOR_BUS_INST,
                                I2C_SENSOR_BUS_CLEAR_FLAGS);
}

static bool wait_bus_idle(void)
{
    uint32_t timeout = I2C_SENSOR_BUS_TIMEOUT;

    while ((DL_I2C_getControllerStatus(I2C_SENSOR_BUS_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0u) {
        if (--timeout == 0u) {
            return false;
        }
    }

    return true;
}

static bool prepare_transfer(void)
{
    if (!wait_bus_idle()) {
        return false;
    }

    clear_transfer_state();
    return true;
}

static bool wait_for_event(uint32_t completeFlag)
{
    uint32_t timeout = I2C_SENSOR_BUS_TIMEOUT;

    while (timeout-- > 0u) {
        uint32_t status = DL_I2C_getRawInterruptStatus(
            I2C_SENSOR_BUS_INST, completeFlag | I2C_SENSOR_BUS_ERROR_FLAGS);

        if ((status & I2C_SENSOR_BUS_ERROR_FLAGS) != 0u) {
            clear_transfer_state();
            return false;
        }
        if ((status & completeFlag) != 0u) {
            DL_I2C_clearInterruptStatus(I2C_SENSOR_BUS_INST, completeFlag);
            return true;
        }
    }

    clear_transfer_state();
    return false;
}

static bool receive_data(uint8_t *data, uint8_t length)
{
    for (uint8_t i = 0u; i < length; i++) {
        uint32_t timeout = I2C_SENSOR_BUS_TIMEOUT;

        while (DL_I2C_isControllerRXFIFOEmpty(I2C_SENSOR_BUS_INST)) {
            uint32_t status = DL_I2C_getRawInterruptStatus(
                I2C_SENSOR_BUS_INST, I2C_SENSOR_BUS_ERROR_FLAGS);

            if ((status & I2C_SENSOR_BUS_ERROR_FLAGS) != 0u ||
                (--timeout == 0u)) {
                clear_transfer_state();
                return false;
            }
        }

        data[i] = DL_I2C_receiveControllerData(I2C_SENSOR_BUS_INST);
    }

    return true;
}

bool I2C_SensorBus_Probe(uint8_t address7bit)
{
    if ((address7bit == 0u) || (address7bit > 0x7Fu) ||
        !prepare_transfer()) {
        return false;
    }

    DL_I2C_startControllerTransfer(I2C_SENSOR_BUS_INST, address7bit,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 0u);
    return wait_for_event(DL_I2C_INTERRUPT_CONTROLLER_STOP);
}

bool I2C_SensorBus_Write(uint8_t address7bit, const uint8_t *data,
                         uint8_t length)
{
    if ((address7bit == 0u) || (address7bit > 0x7Fu) || (data == NULL) ||
        (length == 0u) || (length > I2C_SENSOR_BUS_TX_FIFO_SIZE) ||
        !prepare_transfer()) {
        return false;
    }

    if (DL_I2C_fillControllerTXFIFO(I2C_SENSOR_BUS_INST, (uint8_t *)data,
                                    length) != length) {
        clear_transfer_state();
        return false;
    }

    DL_I2C_startControllerTransfer(I2C_SENSOR_BUS_INST, address7bit,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, length);
    return wait_for_event(DL_I2C_INTERRUPT_CONTROLLER_STOP);
}

bool I2C_SensorBus_WriteRead(uint8_t address7bit, const uint8_t *txData,
                             uint8_t txLength, uint8_t *rxData,
                             uint8_t rxLength)
{
    if ((address7bit == 0u) || (address7bit > 0x7Fu) || (txData == NULL) ||
        (rxData == NULL) || (txLength == 0u) || (rxLength == 0u) ||
        (txLength > I2C_SENSOR_BUS_TX_FIFO_SIZE) || !prepare_transfer()) {
        return false;
    }

    if (DL_I2C_fillControllerTXFIFO(I2C_SENSOR_BUS_INST, (uint8_t *)txData,
                                    txLength) != txLength) {
        clear_transfer_state();
        return false;
    }

    DL_I2C_startControllerTransferAdvanced(
        I2C_SENSOR_BUS_INST, address7bit, DL_I2C_CONTROLLER_DIRECTION_TX,
        txLength, DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE, DL_I2C_CONTROLLER_ACK_DISABLE);
    if (!wait_for_event(DL_I2C_INTERRUPT_CONTROLLER_TX_DONE)) {
        return false;
    }

    DL_I2C_flushControllerRXFIFO(I2C_SENSOR_BUS_INST);
    DL_I2C_clearInterruptStatus(I2C_SENSOR_BUS_INST,
                                I2C_SENSOR_BUS_ERROR_FLAGS |
                                    DL_I2C_INTERRUPT_CONTROLLER_STOP |
                                    DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
    DL_I2C_startControllerTransferAdvanced(
        I2C_SENSOR_BUS_INST, address7bit, DL_I2C_CONTROLLER_DIRECTION_RX,
        rxLength, DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_ENABLE, DL_I2C_CONTROLLER_ACK_DISABLE);

    return receive_data(rxData, rxLength) &&
           wait_for_event(DL_I2C_INTERRUPT_CONTROLLER_STOP);
}
