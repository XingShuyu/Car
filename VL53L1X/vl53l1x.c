/**
 * @file vl53l1x.c
 * @brief VL53L1X synchronous I2C driver implementation.
 */

#include "VL53L1X/vl53l1x.h"

#include "BasicMicroLib/getTime.h"
#include "BasicMicroLib/i2c_sensor_bus.h"
#include <stddef.h>

#define VL53L1X_REG_MODEL_ID                 (0x010Fu)
#define VL53L1X_REG_VHV_CONFIG_TIMEOUT       (0x0008u)
#define VL53L1X_REG_VHV_CONFIG_START         (0x000Bu)
#define VL53L1X_REG_GPIO_HV_MUX_CTRL         (0x0030u)
#define VL53L1X_REG_GPIO_TIO_HV_STATUS       (0x0031u)
#define VL53L1X_REG_ROI_OPTICAL_CENTRE       (0x013Eu)
#define VL53L1X_REG_ROI_CENTRE               (0x007Fu)
#define VL53L1X_REG_ROI_SIZE                 (0x0080u)
#define VL53L1X_REG_INTERRUPT_CLEAR          (0x0086u)
#define VL53L1X_REG_MODE_START               (0x0087u)
#define VL53L1X_REG_RANGE_STATUS             (0x0089u)
#define VL53L1X_REG_DISTANCE_MM              (0x0096u)

#define VL53L1X_MODE_START_CONTINUOUS        (0x40u)
#define VL53L1X_MODE_STOP                     (0x00u)
#define VL53L1X_INTERRUPT_CLEAR_VALUE        (0x01u)
#define VL53L1X_CENTRED_ROI_OPTICAL_CENTRE   (199u)
#define VL53L1X_INIT_READY_TIMEOUT_MS        (500u)

/* Registers 0x002D through 0x0087 from ST/SparkFun default configuration. */
static const uint8_t s_defaultConfiguration[] = {
    0x00u, 0x00u, 0x00u, 0x01u, 0x02u, 0x00u, 0x02u, 0x08u,
    0x00u, 0x08u, 0x10u, 0x01u, 0x01u, 0x00u, 0x00u, 0x00u,
    0x00u, 0xFFu, 0x00u, 0x0Fu, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x20u, 0x0Bu, 0x00u, 0x00u, 0x02u, 0x0Au, 0x21u,
    0x00u, 0x00u, 0x05u, 0x00u, 0x00u, 0x00u, 0x00u, 0xC8u,
    0x00u, 0x00u, 0x38u, 0xFFu, 0x01u, 0x00u, 0x08u, 0x00u,
    0x00u, 0x01u, 0xDBu, 0x0Fu, 0x01u, 0xF1u, 0x0Du, 0x01u,
    0x68u, 0x00u, 0x80u, 0x08u, 0xB8u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x0Fu, 0x89u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x01u, 0x0Fu, 0x0Du, 0x0Eu, 0x0Eu, 0x00u,
    0x00u, 0x02u, 0xC7u, 0xFFu, 0x9Bu, 0x00u, 0x00u, 0x00u,
    0x01u, 0x00u, 0x00u,
};

static bool write_bytes(uint16_t registerAddress, const uint8_t *data,
                        uint8_t length)
{
    uint8_t txData[8];

    if ((data == NULL) || (length == 0u) ||
        (length > (uint8_t)(sizeof(txData) - 2u))) {
        return false;
    }

    txData[0] = (uint8_t)(registerAddress >> 8u);
    txData[1] = (uint8_t)registerAddress;
    for (uint8_t i = 0u; i < length; i++) {
        txData[i + 2u] = data[i];
    }

    return I2C_SensorBus_Write(VL53L1X_I2C_ADDRESS_7BIT, txData,
                               (uint8_t)(length + 2u));
}

static bool write_u8(uint16_t registerAddress, uint8_t value)
{
    return write_bytes(registerAddress, &value, 1u);
}

static bool read_bytes(uint16_t registerAddress, uint8_t *data,
                       uint8_t length)
{
    uint8_t registerBytes[2];

    if ((data == NULL) || (length == 0u)) {
        return false;
    }

    registerBytes[0] = (uint8_t)(registerAddress >> 8u);
    registerBytes[1] = (uint8_t)registerAddress;
    return I2C_SensorBus_WriteRead(VL53L1X_I2C_ADDRESS_7BIT,
                                   registerBytes, sizeof(registerBytes),
                                   data, length);
}

static bool read_u8(uint16_t registerAddress, uint8_t *value)
{
    return read_bytes(registerAddress, value, 1u);
}

static bool read_u16_be(uint16_t registerAddress, uint16_t *value)
{
    uint8_t data[2];

    if ((value == NULL) || !read_bytes(registerAddress, data, sizeof(data))) {
        return false;
    }

    *value = ((uint16_t)data[0] << 8u) | data[1];
    return true;
}

static bool clear_interrupt(void)
{
    return write_u8(VL53L1X_REG_INTERRUPT_CLEAR,
                    VL53L1X_INTERRUPT_CLEAR_VALUE);
}

bool VL53L1X_IsConnected(void)
{
    uint16_t modelId;

    return I2C_SensorBus_Probe(VL53L1X_I2C_ADDRESS_7BIT) &&
           read_u16_be(VL53L1X_REG_MODEL_ID, &modelId) &&
           (modelId == VL53L1X_MODEL_ID);
}

bool VL53L1X_StartRanging(void)
{
    return write_u8(VL53L1X_REG_MODE_START,
                    VL53L1X_MODE_START_CONTINUOUS);
}

bool VL53L1X_StopRanging(void)
{
    return write_u8(VL53L1X_REG_MODE_START, VL53L1X_MODE_STOP);
}

bool VL53L1X_IsDataReady(bool *ready)
{
    uint8_t interruptPolarity;
    uint8_t status;

    if (ready == NULL ||
        !read_u8(VL53L1X_REG_GPIO_HV_MUX_CTRL, &interruptPolarity) ||
        !read_u8(VL53L1X_REG_GPIO_TIO_HV_STATUS, &status)) {
        return false;
    }

    /* GPIO_HV_MUX_CTRL bit 4 is inverted relative to the active polarity. */
    interruptPolarity = (uint8_t)!((interruptPolarity >> 4u) & 0x01u);
    *ready = ((status & 0x01u) == interruptPolarity);
    return true;
}

bool VL53L1X_ReadResult(VL53L1X_Result *result)
{
    uint8_t rangeStatus;
    uint16_t distanceMm;

    if ((result == NULL) ||
        !read_u8(VL53L1X_REG_RANGE_STATUS, &rangeStatus) ||
        !read_u16_be(VL53L1X_REG_DISTANCE_MM, &distanceMm) ||
        !clear_interrupt()) {
        return false;
    }

    result->distanceMm = distanceMm;
    result->rangeStatus = rangeStatus & 0x1Fu;
    return true;
}

bool VL53L1X_SetRoi(uint8_t widthSpads, uint8_t heightSpads)
{
    uint8_t opticalCentre;
    uint8_t size;

    if ((widthSpads < VL53L1X_ROI_MIN_SPADS) ||
        (widthSpads > VL53L1X_ROI_MAX_SPADS) ||
        (heightSpads < VL53L1X_ROI_MIN_SPADS) ||
        (heightSpads > VL53L1X_ROI_MAX_SPADS) ||
        !read_u8(VL53L1X_REG_ROI_OPTICAL_CENTRE, &opticalCentre)) {
        return false;
    }

    /* ST's reference implementation uses SPAD 199 for larger centred ROIs. */
    if ((widthSpads > 10u) || (heightSpads > 10u)) {
        opticalCentre = VL53L1X_CENTRED_ROI_OPTICAL_CENTRE;
    }
    size = (uint8_t)(((heightSpads - 1u) << 4u) | (widthSpads - 1u));

    return write_u8(VL53L1X_REG_ROI_CENTRE, opticalCentre) &&
           write_u8(VL53L1X_REG_ROI_SIZE, size);
}

bool VL53L1X_GetRoi(uint8_t *widthSpads, uint8_t *heightSpads)
{
    uint8_t size;

    if ((widthSpads == NULL) || (heightSpads == NULL) ||
        !read_u8(VL53L1X_REG_ROI_SIZE, &size)) {
        return false;
    }

    *widthSpads = (size & 0x0Fu) + 1u;
    *heightSpads = ((size >> 4u) & 0x0Fu) + 1u;
    return true;
}

bool VL53L1X_Init(void)
{
    bool ready = false;
    uint32_t startMs;

    if (!VL53L1X_IsConnected()) {
        return false;
    }

    for (uint16_t i = 0u; i < sizeof(s_defaultConfiguration); i++) {
        if (!write_u8((uint16_t)(0x002Du + i), s_defaultConfiguration[i])) {
            return false;
        }
    }

    if (!VL53L1X_StartRanging()) {
        return false;
    }

    startMs = getNowMs();
    do {
        if (!VL53L1X_IsDataReady(&ready)) {
            (void)VL53L1X_StopRanging();
            return false;
        }
        if (ready) {
            break;
        }
    } while (getTimeMs(getNowMs(), startMs) < VL53L1X_INIT_READY_TIMEOUT_MS);

    if (!ready || !clear_interrupt() || !VL53L1X_StopRanging()) {
        (void)VL53L1X_StopRanging();
        return false;
    }

    return write_u8(VL53L1X_REG_VHV_CONFIG_TIMEOUT, 0x09u) &&
           write_u8(VL53L1X_REG_VHV_CONFIG_START, 0x00u);
}
