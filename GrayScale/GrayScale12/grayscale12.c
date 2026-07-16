/**
 * @file grayscale12.c
 * @brief 基于 PCA9555 的 NCHD12 十二路灰度循迹驱动。
 */

#include "GrayScale/GrayScale12/grayscale12.h"
#include "BasicMicroLib/i2c_display_bus.h"
#include <stddef.h>

#define PCA9555_INPUT_PORT0_REG       (0x00u)
#define PCA9555_POLARITY_PORT0_REG    (0x04u)
#define PCA9555_CONFIG_PORT0_REG      (0x06u)

typedef struct {
    uint16_t raw12;
    int8_t offset;
} Grayscale12_OffsetEntry_t;

/* bit1 是最右侧传感器，bit12 是最左侧传感器；偏差正负沿用参考工程定义。 */
static const Grayscale12_OffsetEntry_t s_offset10mm[] = {
    {0x0001u, -11}, {0x0003u, -10}, {0x0002u, -9},  {0x0006u, -8},
    {0x0004u, -7},  {0x000Cu, -6},  {0x0008u, -5},  {0x0018u, -4},
    {0x0010u, -3},  {0x0030u, -2},  {0x0020u, -1},  {0x0060u, 0},
    {0x0040u, 1},   {0x00C0u, 2},   {0x0080u, 3},   {0x0180u, 4},
    {0x0100u, 5},   {0x0300u, 6},   {0x0200u, 7},   {0x0600u, 8},
    {0x0400u, 9},   {0x0C00u, 10},  {0x0800u, 11},
};

static const Grayscale12_OffsetEntry_t s_offset20mm[] = {
    {0x0001u, -11}, {0x0003u, -10}, {0x0002u, -9},  {0x0007u, -9},
    {0x0006u, -8},  {0x0004u, -7},  {0x000Eu, -7},  {0x000Cu, -6},
    {0x0008u, -5},  {0x001Cu, -5},  {0x0018u, -4},  {0x0010u, -3},
    {0x0038u, -3},  {0x0030u, -2},  {0x0020u, -1},  {0x0070u, -1},
    {0x0060u, 0},   {0x0040u, 1},   {0x00E0u, 1},   {0x00C0u, 2},
    {0x0080u, 3},   {0x01C0u, 3},   {0x0180u, 4},   {0x0100u, 5},
    {0x0380u, 5},   {0x0300u, 6},   {0x0200u, 7},   {0x0700u, 7},
    {0x0600u, 8},   {0x0400u, 9},   {0x0E00u, 9},   {0x0C00u, 10},
    {0x0800u, 11},
};

static bool write_port_pair(uint8_t startReg, uint8_t port0, uint8_t port1)
{
    uint8_t data[3] = {startReg, port0, port1};

    return I2C_DisplayBus_Write(GRAYSCALE12_I2C_ADDR_7BIT, data,
                                sizeof(data));
}

bool Grayscale12_IsConnected(void)
{
    return I2C_DisplayBus_Probe(GRAYSCALE12_I2C_ADDR_7BIT);
}

bool Grayscale12_Init(void)
{
    if (!Grayscale12_IsConnected()) {
        return false;
    }

    /* 原始电平直接参与偏差表，不在 PCA9555 内做极性翻转。 */
    if (!write_port_pair(PCA9555_POLARITY_PORT0_REG, 0x00u, 0x00u)) {
        return false;
    }

    /* PCA9555 配置位为 1 时对应输入模式。 */
    return write_port_pair(PCA9555_CONFIG_PORT0_REG, 0xFFu, 0xFFu);
}

bool Grayscale12_ReadRaw(uint16_t *raw12)
{
    uint8_t inputReg = PCA9555_INPUT_PORT0_REG;
    uint8_t data[2];

    if (raw12 == NULL ||
        !I2C_DisplayBus_WriteRead(GRAYSCALE12_I2C_ADDR_7BIT, &inputReg,
                                  sizeof(inputReg), data, sizeof(data))) {
        return false;
    }

    *raw12 = (((uint16_t)data[1] << 8u) | data[0]) & GRAYSCALE12_RAW_MASK;
    return true;
}

bool Grayscale12_ReadChannels(bool channels[GRAYSCALE12_CHANNELS])
{
    uint16_t raw12;

    if (channels == NULL || !Grayscale12_ReadRaw(&raw12)) {
        return false;
    }

    for (uint8_t i = 0u; i < GRAYSCALE12_CHANNELS; i++) {
        channels[i] = ((raw12 & ((uint16_t)1u << i)) != 0u);
    }

    return true;
}

bool Grayscale12_GetChannel(uint16_t raw12, uint8_t channel)
{
    if ((channel == 0u) || (channel > GRAYSCALE12_CHANNELS)) {
        return false;
    }

    return ((raw12 & ((uint16_t)1u << (channel - 1u))) != 0u);
}

bool Grayscale12_CalculateLineOffset(uint16_t raw12,
                                     Grayscale12_LineWidth_t lineWidth,
                                     float *offset)
{
    const Grayscale12_OffsetEntry_t *table;
    size_t tableCount;

    if (offset == NULL) {
        return false;
    }

    if (lineWidth == GRAYSCALE12_LINE_WIDTH_10MM) {
        table = s_offset10mm;
        tableCount = sizeof(s_offset10mm) / sizeof(s_offset10mm[0]);
    } else if (lineWidth == GRAYSCALE12_LINE_WIDTH_20MM) {
        table = s_offset20mm;
        tableCount = sizeof(s_offset20mm) / sizeof(s_offset20mm[0]);
    } else {
        return false;
    }

    raw12 &= GRAYSCALE12_RAW_MASK;
    for (size_t i = 0u; i < tableCount; i++) {
        if (table[i].raw12 == raw12) {
            *offset = (float)table[i].offset;
            return true;
        }
    }

    return false;
}

bool Grayscale12_ReadLineOffset(Grayscale12_LineWidth_t lineWidth,
                                uint16_t *raw12, float *offset)
{
    uint16_t raw;

    if (offset == NULL || !Grayscale12_ReadRaw(&raw)) {
        return false;
    }
    if (raw12 != NULL) {
        *raw12 = raw;
    }

    return Grayscale12_CalculateLineOffset(raw, lineWidth, offset);
}
