#include "wdd35d4/wd35d4.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

/* 软件轮询ADC转换完成的超时计数，避免ADC异常时卡死主循环。 */
#define WDD35D4_CONVERSION_TIMEOUT_COUNT (100000UL)

static uint16_t WDD35D4_ClampRaw(uint16_t raw);
static bool WDD35D4_ReadProcessedRaw(float alpha, uint16_t *processed_raw);

/* 驱动内部状态：标定范围、竖直零点、方向和可选软件低通历史值。 */
static uint16_t s_raw_min = WDD35D4_DEFAULT_RAW_MIN;
static uint16_t s_raw_max = WDD35D4_DEFAULT_RAW_MAX;
static uint16_t s_zero_raw = WDD35D4_DEFAULT_ZERO_RAW;
static int8_t s_direction = 1;
static float s_filtered_raw = 0.5f;
static bool s_filter_ready = false;

void WDD35D4_Init(void)
{
    s_raw_min = WDD35D4_DEFAULT_RAW_MIN;
    s_raw_max = WDD35D4_DEFAULT_RAW_MAX;
    s_zero_raw = WDD35D4_DEFAULT_ZERO_RAW;
    s_direction = -1;
    s_filtered_raw = 0.0f;
    s_filter_ready = false;

    DL_ADC12_clearInterruptStatus(
        WDD35D4_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableConversions(WDD35D4_ADC_INST);
}

void WDD35D4_SetCalibration(uint16_t raw_min, uint16_t raw_max)
{
    if (raw_max > raw_min) {
        s_raw_min = raw_min;
        s_raw_max = raw_max;
    }
}

void WDD35D4_ResetCalibration(void)
{
    s_raw_min = WDD35D4_DEFAULT_RAW_MIN;
    s_raw_max = WDD35D4_DEFAULT_RAW_MAX;
}

void WDD35D4_GetCalibration(uint16_t *raw_min, uint16_t *raw_max)
{
    if (raw_min != NULL) {
        *raw_min = s_raw_min;
    }
    if (raw_max != NULL) {
        *raw_max = s_raw_max;
    }
}

bool WDD35D4_CalibrateZero(uint16_t sample_count)
{
    uint32_t raw_sum = 0U;
    uint16_t raw = 0U;

    if (sample_count == 0U) {
        sample_count = WDD35D4_DEFAULT_ZERO_SAMPLES;
    }

    for (uint16_t i = 0U; i < sample_count; i++) {
        if (!WDD35D4_ReadRaw(&raw)) {
            return false;
        }
        raw_sum += raw;
    }

    s_zero_raw = (uint16_t) ((raw_sum + ((uint32_t) sample_count / 2U)) /
                             (uint32_t) sample_count);
    return true;
}

void WDD35D4_SetZeroRaw(uint16_t zero_raw)
{
    s_zero_raw = zero_raw;
}

uint16_t WDD35D4_GetZeroRaw(void)
{
    return s_zero_raw;
}

void WDD35D4_SetDirection(int8_t direction)
{
    s_direction = (direction < 0) ? -1 : 1;
}

int8_t WDD35D4_GetDirection(void)
{
    return s_direction;
}

bool WDD35D4_ReadRaw(uint16_t *raw)
{
    uint32_t timeout = WDD35D4_CONVERSION_TIMEOUT_COUNT;

    if (raw == NULL) {
        return false;
    }

    DL_ADC12_clearInterruptStatus(
        WDD35D4_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_startConversion(WDD35D4_ADC_INST);

    /* 当前ADC配置只使用MEM0，因此等待MEM0结果加载即可。 */
    while ((DL_ADC12_getRawInterruptStatus(
                WDD35D4_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) ==
               0U) &&
           (timeout > 0U)) {
        timeout--;
    }

    if (timeout == 0U) {
        DL_ADC12_stopConversion(WDD35D4_ADC_INST);
        DL_ADC12_enableConversions(WDD35D4_ADC_INST);
        return false;
    }

    *raw = DL_ADC12_getMemResult(WDD35D4_ADC_INST, WDD35D4_ADC_ADCMEM_0);
    DL_ADC12_clearInterruptStatus(
        WDD35D4_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    /* TI例程在每次软件触发采样后重新使能conversion，便于下次触发。 */
    DL_ADC12_enableConversions(WDD35D4_ADC_INST);

    return true;
}

bool WDD35D4_ReadFilteredRaw(float alpha, float *filtered_raw)
{
    uint16_t raw;

    if (filtered_raw == NULL) {
        return false;
    }
    if ((alpha < 0.0f) || (alpha > 1.0f)) {
        alpha = WDD35D4_DEFAULT_FILTER_ALPHA;
    }
    if (!WDD35D4_ReadRaw(&raw)) {
        return false;
    }

    /* alpha=0是明确的直通模式，默认使用该模式减少角度延迟。 */
    if (alpha <= WDD35D4_FILTER_DISABLED) {
        *filtered_raw = (float) raw;
        return true;
    }

    if (!s_filter_ready) {
        s_filtered_raw = (float) raw;
        s_filter_ready = true;
    } else {
        s_filtered_raw = (s_filtered_raw * (1.0f - alpha)) +
                         ((float) raw * alpha);
    }

    *filtered_raw = s_filtered_raw;
    return true;
}

bool WDD35D4_ReadVoltage(float *voltage)
{
    uint16_t raw;

    if (voltage == NULL) {
        return false;
    }
    if (!WDD35D4_ReadRaw(&raw)) {
        return false;
    }

    *voltage = WDD35D4_RawToVoltage(raw);
    return true;
}

bool WDD35D4_ReadAngleDeg(float *angle_deg)
{
    uint16_t raw;

    if (angle_deg == NULL) {
        return false;
    }
    if (!WDD35D4_ReadProcessedRaw(WDD35D4_DEFAULT_FILTER_ALPHA, &raw)) {
        return false;
    }

    *angle_deg = WDD35D4_RawToAngleDeg(raw);
    return true;
}

bool WDD35D4_ReadSignedAngleDeg(float *signed_angle_deg)
{
    uint16_t raw;

    if (signed_angle_deg == NULL) {
        return false;
    }
    if (!WDD35D4_ReadProcessedRaw(WDD35D4_DEFAULT_FILTER_ALPHA, &raw)) {
        return false;
    }

    *signed_angle_deg = WDD35D4_RawToSignedAngleDeg(raw);
    return true;
}

bool WDD35D4_ReadData(WDD35D4_Data_t *data)
{
    uint16_t raw;

    if (data == NULL) {
        return false;
    }
    if (!WDD35D4_ReadProcessedRaw(WDD35D4_DEFAULT_FILTER_ALPHA, &raw)) {
        return false;
    }

    data->raw = raw;
    data->voltage = WDD35D4_RawToVoltage(raw);
    data->angle_deg = WDD35D4_RawToAngleDeg(raw);
    data->signed_angle_deg = WDD35D4_RawToSignedAngleDeg(raw);

    return true;
}

float WDD35D4_RawToVoltage(uint16_t raw)
{
    return ((float) raw * WDD35D4_ADC_ADCMEM_0_REF_VOLTAGE_V) /
           (float) WDD35D4_ADC_MAX_RAW;
}

float WDD35D4_RawToAngleDeg(uint16_t raw)
{
    uint16_t clamped;
    uint16_t span;

    if (s_raw_max <= s_raw_min) {
        return 0.0f;
    }

    clamped = WDD35D4_ClampRaw(raw);
    span = (uint16_t) (s_raw_max - s_raw_min);
    return ((float) (clamped - s_raw_min) * WDD35D4_ELECTRICAL_ANGLE_DEG) /
           (float) span;
}

float WDD35D4_RawToSignedAngleDeg(uint16_t raw)
{
    float raw_delta;
    float raw_span;
    uint16_t clamped;

    if (s_raw_max <= s_raw_min) {
        return 0.0f;
    }

    clamped = WDD35D4_ClampRaw(raw);
    raw_delta = (float) ((int32_t) clamped - (int32_t) s_zero_raw);
    raw_span = (float) (s_raw_max - s_raw_min);

    return raw_delta * WDD35D4_ELECTRICAL_ANGLE_DEG *
           (float) s_direction / raw_span;
}

static uint16_t WDD35D4_ClampRaw(uint16_t raw)
{
    if (raw < s_raw_min) {
        return s_raw_min;
    }
    if (raw > s_raw_max) {
        return s_raw_max;
    }
    return raw;
}

static bool WDD35D4_ReadProcessedRaw(float alpha, uint16_t *processed_raw)
{
    float filtered_raw;

    if (processed_raw == NULL) {
        return false;
    }
    if (!WDD35D4_ReadFilteredRaw(alpha, &filtered_raw)) {
        return false;
    }

    *processed_raw = (uint16_t) (filtered_raw + 0.5f);
    return true;
}
