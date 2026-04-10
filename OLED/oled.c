/**
 * @file oled.c
 * @brief OLED 商家驱动实现（适配版）
 * @note 依赖 display.c 提供的 I2C 驾驶和显存缓冲区
 */

#include "oled.h"
#include <stdlib.h> // 用于 abs

// 如果 display.c 中定义了 framebuffer，这里就不要重复定义，使用 extern
// 但如果商家代码必须自己管理显存，则需要在这里定义，并在最后 memcpy 到 display 的显存
// 这里采用最简单的方案：直接引用 display.c 的显存

// 如果编译报错 "undefined reference to 'framebuffer'"，请取消下面这行的注释
// uint8_t framebuffer[OLED_WIDTH * OLED_HEIGHT / 8]; 

/**
 * @brief 画点（操作显存）
 * @param x 横坐标
 * @param y 纵坐标
 */
void OLED_DrawPoint(uint8_t x, uint8_t y) 
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    
    // 计算显存地址 (页寻址模式)
    // OLED_PAGE_SIZE = OLED_WIDTH (通常为 128)
    uint16_t index = x + (y / 8) * OLED_WIDTH; 
    
    // 设置位
    framebuffer[index] |= (1 << (y % 8)); 
}

/**
 * @brief 清除点
 * @param x 横坐标
 * @param y 纵坐标
 */
void OLED_ClearPoint(uint8_t x, uint8_t y) 
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    
    uint16_t index = x + (y / 8) * OLED_WIDTH; 
    framebuffer[index] &= ~(1 << (y % 8)); 
}

/**
 * @brief 画线 (Bresenham 算法)
 */
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
    int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1; 
    int err = dx + dy, e2;

    for(;;){
        OLED_DrawPoint(x1, y1);
        if (x1 == x2 && y1 == y2) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x1 += sx; }
        if (e2 <= dx) { err += dx; y1 += sy; }
    }
}

/**
 * @brief 画圆 (Midpoint Circle Algorithm)
 */
void OLED_DrawCircle(uint8_t x0, uint8_t y0, uint8_t r)
{
    int f = 1 - r;
    int ddF_x = 1;
    int ddF_y = -2 * r;
    int x = 0;
    int y = r;

    OLED_DrawPoint(x0, y0 + r);
    OLED_DrawPoint(x0, y0 - r);
    OLED_DrawPoint(x0 + r, y0);
    OLED_DrawPoint(x0 - r, y0);

    while(x < y)
    {
        if(f >= 0)
        {
            y--; 
            ddF_y += 2; 
            f += ddF_y;
        }
        x++;
        ddF_x += 2; 
        f += ddF_x;   

        OLED_DrawPoint(x0 + x, y0 + y);
        OLED_DrawPoint(x0 - x, y0 + y);
        OLED_DrawPoint(x0 + x, y0 - y);
        OLED_DrawPoint(x0 - x, y0 - y);

        OLED_DrawPoint(x0 + y, y0 + x);
        OLED_DrawPoint(x0 - y, y0 + x);
        OLED_DrawPoint(x0 + y, y0 - x);
        OLED_DrawPoint(x0 - y, y0 - x);
    }
}

// 这里可以添加商家特有的其他函数（如显示图片）
// void OLED_DrawBMP(...) { ... }