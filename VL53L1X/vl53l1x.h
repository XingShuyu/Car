/**
 * @file vl53l1x.h
 * @brief ST VL53L1X ToF distance-sensor driver on I2C0 (PA28/PA31).
 *
 * The sensor uses a 7-bit I2C address and 16-bit big-endian register
 * addresses. All functions are synchronous and must be called from main-loop
 * context after SYSCFG_DL_init().
 */

#ifndef VL53L1X_VL53L1X_H_
#define VL53L1X_VL53L1X_H_

#include <stdbool.h>
#include <stdint.h>

#define VL53L1X_I2C_ADDRESS_7BIT       (0x29u)
#define VL53L1X_MODEL_ID               (0xEACCu)
#define VL53L1X_ROI_MIN_SPADS          (4u)
#define VL53L1X_ROI_MAX_SPADS          (16u)

/** A completed ranging sample read from the VL53L1X. */
typedef struct VL53L1X_Result {
    uint16_t distanceMm;
    uint8_t rangeStatus;
} VL53L1X_Result;

/**
 * @brief Probe the sensor and verify its model ID.
 */
bool VL53L1X_IsConnected(void);

/**
 * @brief Load the ST default configuration and prepare the sensor for use.
 *
 * Initialization performs and acknowledges one ranging cycle, then leaves
 * ranging stopped. Call VL53L1X_StartRanging() before reading measurements.
 */
bool VL53L1X_Init(void);

/** Start or stop continuous ranging. */
bool VL53L1X_StartRanging(void);
bool VL53L1X_StopRanging(void);

/**
 * @brief Check whether a new measurement is available.
 * @param ready Receives true when a new result can be read.
 */
bool VL53L1X_IsDataReady(bool *ready);

/**
 * @brief Read a new measurement and acknowledge it in the sensor.
 *
 * The result's rangeStatus is the low five bits of register 0x0089. The
 * interrupt is cleared only after both status and distance are read.
 */
bool VL53L1X_ReadResult(VL53L1X_Result *result);

/**
 * @brief Set the centred ROI size in SPADs.
 *
 * Width and height must both be in the inclusive range 4 to 16.
 */
bool VL53L1X_SetRoi(uint8_t widthSpads, uint8_t heightSpads);

/** Read the programmed ROI size in SPADs. */
bool VL53L1X_GetRoi(uint8_t *widthSpads, uint8_t *heightSpads);

#endif /* VL53L1X_VL53L1X_H_ */
