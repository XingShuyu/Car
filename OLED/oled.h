/**
 * @file oled.h
 * @brief SSD1306 OLED low-level driver.
 *
 * This module owns the SSD1306 I2C transfer logic and the framebuffer.
 * Higher-level text formatting is implemented in display.c.
 */

#ifndef OLED_H_
#define OLED_H_

#include <stdbool.h>
#include <stdint.h>

#define OLED_WIDTH      128u
#define OLED_HEIGHT     64u
#define OLED_PAGE_COUNT (OLED_HEIGHT / 8u)
#define OLED_BUFFER_SIZE (OLED_WIDTH * OLED_PAGE_COUNT)

bool OLED_Init(void);
bool OLED_IsReady(void);
void OLED_Clear(void);
bool OLED_Update(void);
bool OLED_UpdatePage(uint8_t page);

void OLED_SetPixel(uint8_t x, uint8_t y, bool on);
void OLED_DrawPoint(uint8_t x, uint8_t y);
void OLED_ClearPoint(uint8_t x, uint8_t y);
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void OLED_DrawCircle(uint8_t x0, uint8_t y0, uint8_t r);

uint8_t *OLED_GetBuffer(void);

#endif /* OLED_H_ */
