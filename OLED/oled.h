/**
 * @file oled.h
 * @brief OLED 商家库文件（适配层）
 * @note 此文件仅包含绘图逻辑，底层 I2C 依赖于 display.c
 */

#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

// ----------------- 屏幕参数配置 -----------------
// 请根据商家资料修改此处，通常为 128x64 或 128x32
#define OLED_WIDTH  128
#define OLED_HEIGHT 64

// ----------------- 绘图接口 -----------------
// 这些函数由商家代码实现，用于操作显存
void OLED_DrawPoint(uint8_t x, uint8_t y); // 画点
void OLED_DrawCircle(uint8_t x0, uint8_t y0, uint8_t r); // 画圆
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2); // 画线

// ----------------- 兼容接口 -----------------
// 这些宏或函数是为了让商家代码能“看到”display.c 中的显存
// 必须与 display.c 中的 framebuffer 定义一致
extern uint8_t framebuffer[]; // 引用 display.c 中的显存

#endif