/**
 * @file grayscale12.h
 * @brief NCHD12 十二路灰度循迹模块驱动（PCA9555，I2C1 共用 OLED 总线）。
 */

#ifndef GRAYSCALE12_H_
#define GRAYSCALE12_H_

#include <stdbool.h>
#include <stdint.h>

#define GRAYSCALE12_CHANNELS        (12u)
#define GRAYSCALE12_RAW_MASK        (0x0FFFu)
#define GRAYSCALE12_I2C_ADDR_7BIT   (0x20u)

/** 支持的黑线宽度偏差表。 */
typedef enum {
    GRAYSCALE12_LINE_WIDTH_10MM = 0,
    GRAYSCALE12_LINE_WIDTH_20MM,
} Grayscale12_LineWidth_t;

/**
 * 初始化 PCA9555：关闭极性反转并将两个端口配置为输入。
 *
 * 调用前必须已经执行 SYSCFG_DL_init()。返回 false 表示设备未应答或总线传输失败。
 */
bool Grayscale12_Init(void);

/** 检测默认 7 位地址 0x20 的 NCHD12 是否在线。 */
bool Grayscale12_IsConnected(void);

/**
 * 读取 12 路原始状态。
 *
 * PCA9555 的 INPUT_PORT0/1 按低字节在前读取，只返回低 12 位。bit1 对应
 * raw12 的 bit0（传感器右侧），bit12 对应 bit11（传感器左侧）。
 */
bool Grayscale12_ReadRaw(uint16_t *raw12);

/**
 * 读取所有通道。channels[0] 对应 bit1（右侧），channels[11] 对应 bit12（左侧）。
 */
bool Grayscale12_ReadChannels(bool channels[GRAYSCALE12_CHANNELS]);

/**
 * 获取指定通道状态，channel 的有效范围为 1 至 12。
 * 无效通道返回 false。
 */
bool Grayscale12_GetChannel(uint16_t raw12, uint8_t channel);

/**
 * 按参考工程的 10 mm 或 20 mm 黑线偏差表换算。
 *
 * 返回 true 代表 raw12 命中偏差表；未命中（例如丢线、路口）时返回 false，
 * 调用方可根据自己的控制策略保持上一帧输出或进行特殊处理。
 */
bool Grayscale12_CalculateLineOffset(uint16_t raw12,
                                     Grayscale12_LineWidth_t lineWidth,
                                     float *offset);

/**
 * 读取一帧并计算偏差。raw12 可为 NULL；返回值同时表示 I2C 读取成功且状态
 * 命中偏差表。
 */
bool Grayscale12_ReadLineOffset(Grayscale12_LineWidth_t lineWidth,
                                uint16_t *raw12, float *offset);

#endif /* GRAYSCALE12_H_ */
