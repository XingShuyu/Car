/**
 * @file Grayscale_Scan.c
 * @brief 8/12 路灰度模块的统一循迹与路口适配实现。
 */

#include "GrayScale/Grayscale_Scan.h"
#include "GrayScale/GrayScale8/Grayscale_Scan.h"
#include "GrayScale/GrayScale12/grayscale12_control.h"
#include <stddef.h>

#define GRAYSCALE12_COMPAT_KP        (1070.0f)
#define GRAYSCALE12_COMPAT_KI        (0.0f)
#define GRAYSCALE12_COMPAT_KD        (0.0f)
#define GRAYSCALE12_COMPAT_I_MAX     (100000.0f)
#define GRAYSCALE12_COMPAT_PERIOD_MS (10.0f)
#define GRAYSCALE12_COMPAT_IRR_SCALE (1.5f)

static PID s_pid12 = {
    .p = GRAYSCALE12_COMPAT_KP,
    .i = GRAYSCALE12_COMPAT_KI,
    .d = GRAYSCALE12_COMPAT_KD,
    .i_Max = GRAYSCALE12_COMPAT_I_MAX,
    .saved_i = 0.0f,
    .t = GRAYSCALE12_COMPAT_PERIOD_MS,
};

static Grayscale12_LineController_t s_controller12;
static bool s_controller12Ready = false;
static float s_irrScale12 = GRAYSCALE12_COMPAT_IRR_SCALE;

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

static void ensure_controller12(void)
{
    if (!s_controller12Ready || (s_controller12.pid != &s_pid12)) {
        (void)Grayscale12_LineController_Init(&s_controller12, &s_pid12);
        s_controller12Ready = true;
    }
}

static uint16_t channels_to_raw12(
    const bool sensorValues[GRAYSCALE_SENSOR_CHANNELS])
{
    uint16_t raw12 = 0u;

    for (uint8_t i = 0u; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if (sensorValues[i]) {
            raw12 |= (uint16_t)1u << i;
        }
    }

    return raw12;
}

float Grayscale_Line(PID *pid,
                     bool sensorValues[GRAYSCALE_SENSOR_CHANNELS])
{
    if (sensorValues == NULL) {
        return 0.0f;
    }

    if (Grayscale_GetActiveDriver() == GrayscaleDriver8) {
        bool values8[GRAYSCALE8_SENSOR_CHANNELS];
        float irr = Grayscale8_Line(pid, values8);

        map_eight_channels(values8, sensorValues);
        return irr;
    }

    {
        bool lineDetected;

        ensure_controller12();
        Grayscale_Sensor_Read_All(sensorValues);
        return Grayscale12_LineFromRaw(
                   &s_controller12, channels_to_raw12(sensorValues),
                   &lineDetected) *
               s_irrScale12;
    }
}

bool Grayscale_Cross(bool sensorValues[GRAYSCALE_SENSOR_CHANNELS], int status)
{
    if (sensorValues == NULL) {
        return false;
    }

    if (Grayscale_GetActiveDriver() == GrayscaleDriver8) {
        bool values8[GRAYSCALE8_SENSOR_CHANNELS];
        bool result = Grayscale8_Cross(values8, status);

        map_eight_channels(values8, sensorValues);
        return result;
    }

    Grayscale_Sensor_Read_All(sensorValues);
    if (!Grayscale_LastReadOk()) {
        return false;
    }

    if (status == 1) {
        return sensorValues[0] && sensorValues[1] && sensorValues[2] &&
               sensorValues[3] && !sensorValues[11];
    }
    if (status == 2) {
        return sensorValues[8] && sensorValues[9] && sensorValues[10] &&
               sensorValues[11] && !sensorValues[0];
    }
    if (status == 0) {
        for (uint8_t i = 2u; i <= 9u; i++) {
            if (!sensorValues[i]) {
                return false;
            }
        }
        return true;
    }

    return false;
}

void Grayscale_Zero(bool sensorValues[GRAYSCALE_SENSOR_CHANNELS])
{
    if (sensorValues == NULL) {
        return;
    }

    if (Grayscale_GetActiveDriver() == GrayscaleDriver8) {
        bool values8[GRAYSCALE8_SENSOR_CHANNELS];

        Grayscale8_Zero(values8);
        map_eight_channels(values8, sensorValues);
        return;
    }

    ensure_controller12();
    Grayscale12_LineController_Reset(&s_controller12);
    clear_channels(sensorValues);
    sensorValues[5] = true;
    sensorValues[6] = true;
}

int Grayscale_OnlineNum(bool sensorValues[GRAYSCALE_SENSOR_CHANNELS])
{
    if (sensorValues == NULL) {
        return 0;
    }

    if (Grayscale_GetActiveDriver() == GrayscaleDriver8) {
        bool values8[GRAYSCALE8_SENSOR_CHANNELS];
        int count = Grayscale8_OnlineNum(values8);

        map_eight_channels(values8, sensorValues);
        return count;
    }

    Grayscale_Sensor_Read_All(sensorValues);
    if (!Grayscale_LastReadOk()) {
        return 0;
    }

    {
        int count = 0;

        for (uint8_t i = 0u; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
            if (sensorValues[i]) {
                count++;
            }
        }
        return count;
    }
}

bool Grayscale_GetTurnControl(Grayscale_TurnDirection_t direction,
                              bool *reached, float *speedScale)
{
    bool values[GRAYSCALE_SENSOR_CHANNELS];
    float scale = 1.0f;
    bool isReached = false;

    Grayscale_Sensor_Read_All(values);
    if (!Grayscale_LastReadOk()) {
        if (reached != NULL) {
            *reached = false;
        }
        if (speedScale != NULL) {
            *speedScale = scale;
        }
        return false;
    }

    if (direction == GrayscaleTurnRight) {
        if (Grayscale_GetActiveDriver() == GrayscaleDriver12) {
            isReached = values[6];       /* P7 */
            if (values[11]) {
                scale = 0.5f;            /* P12 */
            } else if (values[10]) {
                scale = 0.3f;            /* P11 */
            } else if (values[9]) {
                scale = 0.1f;            /* P10 */
            }
        } else {
            isReached = values[6];       /* old channel 4 */
            if (values[9]) {
                scale = 0.5f;            /* old channel 7 */
            } else if (values[8]) {
                scale = 0.3f;            /* old channel 6 */
            } else if (values[7]) {
                scale = 0.1f;            /* old channel 5 */
            }
        }
    } else {
        if (Grayscale_GetActiveDriver() == GrayscaleDriver12) {
            isReached = values[5];       /* P6 */
            if (values[0]) {
                scale = 0.5f;            /* P1 */
            } else if (values[1]) {
                scale = 0.3f;            /* P2 */
            } else if (values[2]) {
                scale = 0.1f;            /* P3 */
            }
        } else {
            isReached = values[5];       /* old channel 3 */
            if (values[2]) {
                scale = 0.5f;            /* old channel 0 */
            } else if (values[3]) {
                scale = 0.3f;            /* old channel 1 */
            } else if (values[4]) {
                scale = 0.1f;            /* old channel 2 */
            }
        }
    }

    if (reached != NULL) {
        *reached = isReached;
    }
    if (speedScale != NULL) {
        *speedScale = scale;
    }
    return true;
}

bool Grayscale_Set12Pid(const PID *pid)
{
    if (pid == NULL) {
        return false;
    }

    s_pid12 = *pid;
    s_pid12.saved_i = 0.0f;
    s_controller12Ready = false;
    ensure_controller12();
    return true;
}

const PID *Grayscale_Get12Pid(void)
{
    return &s_pid12;
}

bool Grayscale_Set12IrrScale(float scale)
{
    if (scale == 0.0f) {
        return false;
    }

    s_irrScale12 = scale;
    return true;
}

float Grayscale_Get12IrrScale(void)
{
    return s_irrScale12;
}
