/**
 * @file grayscale_sensor.c
 * @brief 8/12 路灰度传感器启动选择与统一通道映射。
 */

#include "GrayScale/grayscale_sensor.h"
#include "GrayScale/GrayScale8/grayscale_sensor.h"
#include "GrayScale/GrayScale12/grayscale12.h"
#include <stddef.h>

static Grayscale_Driver_t s_activeDriver = GrayscaleDriver8;
static bool s_lastReadOk = false;

static void clear_channels(bool sensorValues[GRAYSCALE_SENSOR_CHANNELS])
{
    for (uint8_t i = 0u; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        sensorValues[i] = false;
    }
}

static void map_eight_channels(const bool source[GRAYSCALE8_SENSOR_CHANNELS],
                               bool destination[GRAYSCALE_SENSOR_CHANNELS])
{
    clear_channels(destination);

    for (uint8_t i = 0u; i < GRAYSCALE8_SENSOR_CHANNELS; i++) {
        destination[i + 2u] = source[i];
    }
}

bool Grayscale_Init(void)
{
    if (Grayscale12_Init()) {
        s_activeDriver = GrayscaleDriver12;
        s_lastReadOk = true;
        return true;
    }

    Grayscale8_Sensor_Init();
    s_activeDriver = GrayscaleDriver8;
    s_lastReadOk = true;
    return true;
}

Grayscale_Driver_t Grayscale_GetActiveDriver(void)
{
    return s_activeDriver;
}

bool Grayscale_LastReadOk(void)
{
    return s_lastReadOk;
}

void Grayscale_Sensor_Init(void)
{
    (void)Grayscale_Init();
}

void Grayscale_Sensor_Read_All(bool sensorValues[GRAYSCALE_SENSOR_CHANNELS])
{
    if (sensorValues == NULL) {
        s_lastReadOk = false;
        return;
    }

    if (s_activeDriver == GrayscaleDriver12) {
        bool values12[GRAYSCALE_SENSOR_CHANNELS] = {false};

        s_lastReadOk = Grayscale12_ReadChannels(values12);
        if (s_lastReadOk) {
            for (uint8_t i = 0u; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
                sensorValues[i] = values12[i];
            }
        } else {
            clear_channels(sensorValues);
        }
        return;
    }

    {
        bool values8[GRAYSCALE8_SENSOR_CHANNELS];

        Grayscale8_Sensor_Read_All(values8);
        map_eight_channels(values8, sensorValues);
        s_lastReadOk = true;
    }
}

void Grayscale_Sensor_Read_Main(bool sensorValues[GRAYSCALE_SENSOR_CHANNELS])
{
    bool values[GRAYSCALE_SENSOR_CHANNELS];

    if (sensorValues == NULL) {
        s_lastReadOk = false;
        return;
    }

    Grayscale_Sensor_Read_All(values);
    for (uint8_t i = 4u; i <= 7u; i++) {
        sensorValues[i] = values[i];
    }
}

void Grayscale_Sensor_Read_Other(bool sensorValues[GRAYSCALE_SENSOR_CHANNELS])
{
    bool values[GRAYSCALE_SENSOR_CHANNELS];

    if (sensorValues == NULL) {
        s_lastReadOk = false;
        return;
    }

    Grayscale_Sensor_Read_All(values);
    for (uint8_t i = 0u; i < 4u; i++) {
        sensorValues[i] = values[i];
    }
    for (uint8_t i = 8u; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        sensorValues[i] = values[i];
    }
}

bool Grayscale_Sensor_Read_Single(uint8_t channel)
{
    bool values[GRAYSCALE_SENSOR_CHANNELS];

    if (channel >= GRAYSCALE_SENSOR_CHANNELS) {
        s_lastReadOk = false;
        return false;
    }

    Grayscale_Sensor_Read_All(values);
    return s_lastReadOk && values[channel];
}

bool _read_channel_stable(uint8_t channel)
{
    return Grayscale_Sensor_Read_Single(channel);
}
