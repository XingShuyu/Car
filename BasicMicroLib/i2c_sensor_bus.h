/**
 * @file i2c_sensor_bus.h
 * @brief I2C0 sensor-bus synchronous access helpers.
 *
 * I2C0 is connected to PA28 (SDA) and PA31 (SCL). It is shared by the IMU
 * drivers and VL53L1X, so transfers must be initiated only from main-loop
 * context, never from an interrupt handler.
 */

#ifndef I2C_SENSOR_BUS_H_
#define I2C_SENSOR_BUS_H_

#include <stdbool.h>
#include <stdint.h>

/** Detect whether a 7-bit I2C device responds on I2C0. */
bool I2C_SensorBus_Probe(uint8_t address7bit);

/**
 * Write data to a 7-bit I2C device on I2C0.
 * A single transfer is limited to the MSPM0 controller's 8-byte TX FIFO.
 */
bool I2C_SensorBus_Write(uint8_t address7bit, const uint8_t *data,
                         uint8_t length);

/** Write data then read through a repeated START on I2C0. */
bool I2C_SensorBus_WriteRead(uint8_t address7bit, const uint8_t *txData,
                             uint8_t txLength, uint8_t *rxData,
                             uint8_t rxLength);

#endif /* I2C_SENSOR_BUS_H_ */
