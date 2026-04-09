/**
 * @file    display.h
 * @brief   OLED 显示屏驱动（SSD1306, I2C 接口）
 * @details 提供初始化、清屏、显示字符串、显示速度等功能
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 初始化显示屏（必须在上电后调用）
 */
void Display_Init(void);

/**
 * @brief 清屏（全黑）
 */
void Display_Clear(void);

/**
 * @brief 在指定位置显示字符串（支持 ASCII 32~126）
 * @param row    行号（0~7，每行 8 像素）
 * @param col    列号（0~20，每个字符宽 6 像素）
 * @param str    要显示的字符串
 */
void Display_ShowString(uint8_t row, uint8_t col, const char *str);

/**
 * @brief 显示当前速度（单位：cm/s）
 * @param speed  速度值（浮点数，保留一位小数）
 */
void Display_ShowSpeed(float speed);

/**
 * @brief 显示自定义信息（例如角度、距离等）
 * @param label  标签字符串
 * @param value  数值
 */
void Display_ShowValue(const char *label, float value);

/**
 * @brief 刷新显示（如果需要立即输出，此函数通常内嵌在显示函数中）
 */
void Display_Update(void);

#endif /* DISPLAY_H_ */