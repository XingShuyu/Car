/**
 * @file    display.c (修正版)
 * @brief   SSD1306 OLED 驱动实现（使用 DriverLib 标准 I2C API）
 * @note    依赖 SysConfig 生成的 I2C2 实例（I2C_LED_panel）
 */

#include "display.h"
#include "ti_msp_dl_config.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* 显示屏参数 -----------------------------------------------------------*/
#define OLED_ADDR           0x78    // SSD1306 I2C 地址（7位地址 0x3C，左移一位得 0x78）
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_PAGE_NUM       8       // 64/8 = 8 页

/* I2C 实例（使用你配置的 I2C2，即 I2C_LED_panel）*/
#define DISPLAY_I2C_INST    I2C_LED_display_INST
#define I2C_TIMEOUT_MS      1000u

/* 内部缓冲区：显存（1 字节/像素，共 1024 字节）*/
static uint8_t framebuffer[OLED_WIDTH * OLED_HEIGHT / 8];

/* 字库：6x8 像素 ASCII 字符（部分示例，实际需补全）*/
static const uint8_t font6x8[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, // 空格
    {0x00,0x00,0x5F,0x00,0x00,0x00}, // !
    // ... 此处需补全所有 ASCII 字符的点阵数据
};

/* ------------------- I2C 通信底层函数 ------------------- */
static bool i2c_wait_idle(void)
{
    uint32_t timeout = I2C_TIMEOUT_MS * 1000u;
    while (DL_I2C_getControllerStatus(DISPLAY_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--timeout == 0u) return false;
    }
    return true;
}

static bool i2c_wait_done(void)
{
    uint32_t timeout = I2C_TIMEOUT_MS * 1000u;
    /* Wait for STOP condition or NACK */
    while (!DL_I2C_getRawInterruptStatus(
                DISPLAY_I2C_INST,
                DL_I2C_INTERRUPT_CONTROLLER_STOP |
                DL_I2C_INTERRUPT_CONTROLLER_NACK)) {
        if (--timeout == 0u) return false;
    }
    /* Check for NACK */
    if (DL_I2C_getRawInterruptStatus(DISPLAY_I2C_INST,
            DL_I2C_INTERRUPT_CONTROLLER_NACK)) {
        DL_I2C_clearInterruptStatus(DISPLAY_I2C_INST,
            DL_I2C_INTERRUPT_CONTROLLER_NACK |
            DL_I2C_INTERRUPT_CONTROLLER_STOP);
        return false;
    }
    DL_I2C_clearInterruptStatus(DISPLAY_I2C_INST,
        DL_I2C_INTERRUPT_CONTROLLER_STOP);
    return true;
}

static bool i2c_write_bytes(uint8_t *data, uint16_t len) {
    if (!i2c_wait_idle()) return false;

    DL_I2C_clearInterruptStatus(DISPLAY_I2C_INST,
        DL_I2C_INTERRUPT_CONTROLLER_NACK |
        DL_I2C_INTERRUPT_CONTROLLER_STOP);

    /* 清空并填充 TX FIFO */
    DL_I2C_flushControllerTXFIFO(DISPLAY_I2C_INST);
    for (uint16_t i = 0; i < len; i++) {
        DL_I2C_fillControllerTXFIFO(DISPLAY_I2C_INST, &data[i], 1);
    }

    /* 启动传输 */
    DL_I2C_startControllerTransfer(
        DISPLAY_I2C_INST,
        OLED_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        len);

    return i2c_wait_done();
}

static bool i2c_write_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};   // 控制字节：0x00 表示命令
    return i2c_write_bytes(buf, 2);
}

static bool i2c_write_data(uint8_t *data, uint16_t len) {
    uint8_t *buf = malloc(len + 1);
    if (!buf) return false;
    buf[0] = 0x40;  // 控制字节：0x40 表示数据
    memcpy(buf + 1, data, len);
    bool result = i2c_write_bytes(buf, len + 1);
    free(buf);
    return result;
}

/* ------------------- 硬件初始化 ------------------- */
static void oled_write_cmd(uint8_t cmd) {
    i2c_write_cmd(cmd);
}

static void oled_set_page(uint8_t page, uint8_t col) {
    oled_write_cmd(0xB0 + page);          // 设置页地址
    oled_write_cmd(0x00 + (col & 0x0F));  // 低四位列地址
    oled_write_cmd(0x10 + ((col >> 4) & 0x0F)); // 高四位列地址
}

void Display_Init(void) {
    // 等待 I2C 稳定
    for (volatile int i = 0; i < 10000; i++);
    
    // SSD1306 初始化序列
    oled_write_cmd(0xAE); // 关闭显示
    oled_write_cmd(0xD5); oled_write_cmd(0x80); // 时钟分频
    oled_write_cmd(0xA8); oled_write_cmd(0x3F); // 复用率
    oled_write_cmd(0xD3); oled_write_cmd(0x00); // 显示偏移
    oled_write_cmd(0x40); // 起始行
    oled_write_cmd(0x8D); oled_write_cmd(0x14); // 电荷泵使能
    oled_write_cmd(0x20); oled_write_cmd(0x00); // 内存寻址模式
    oled_write_cmd(0xA1); // 段重映射（列地址 127->0）
    oled_write_cmd(0xC8); // 行扫描方向反向
    oled_write_cmd(0xDA); oled_write_cmd(0x12); // COM 引脚配置
    oled_write_cmd(0x81); oled_write_cmd(0xCF); // 对比度
    oled_write_cmd(0xD9); oled_write_cmd(0xF1); // 预充电周期
    oled_write_cmd(0xDB); oled_write_cmd(0x40); // VCOM 电压
    oled_write_cmd(0xA4); // 全亮恢复
    oled_write_cmd(0xA6); // 正常显示（非反色）
    oled_write_cmd(0xAF); // 开启显示
    Display_Clear();
}

void Display_Clear(void) {
    memset(framebuffer, 0x00, sizeof(framebuffer));
    // 将显存全部写入 OLED
    for (uint8_t page = 0; page < OLED_PAGE_NUM; page++) {
        oled_set_page(page, 0);
        i2c_write_data(&framebuffer[page * OLED_WIDTH], OLED_WIDTH);
    }
}

/* ------------------- 绘图函数 ------------------- */
static void draw_pixel(uint8_t x, uint8_t y, uint8_t color) {
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    uint16_t index = x + (y / 8) * OLED_WIDTH;
    if (color) framebuffer[index] |= (1 << (y % 8));
    else framebuffer[index] &= ~(1 << (y % 8));
}

/* 显示一个字符（6x8 点阵）*/
static void draw_char(uint8_t x, uint8_t y, char ch) {
    if (ch < 0x20 || ch > 0x7E) ch = 0x20; // 只显示 ASCII
    const uint8_t *glyph = font6x8[ch - 0x20];
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t line = glyph[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (line & (1 << j))
                draw_pixel(x + i, y + j, 1);
        }
    }
}

/* 显示字符串（自动换行）*/
void Display_ShowString(uint8_t row, uint8_t col, const char *str) {
    uint8_t x = col * 6;
    uint8_t y = row * 8;
    while (*str && x < OLED_WIDTH) {
        draw_char(x, y, *str++);
        x += 6;
        if (x + 6 > OLED_WIDTH) { // 换行
            x = 0;
            y += 8;
            if (y >= OLED_HEIGHT) break;
        }
    }
    // 更新 OLED 显示
    for (uint8_t page = 0; page < OLED_PAGE_NUM; page++) {
        oled_set_page(page, 0);
        i2c_write_data(&framebuffer[page * OLED_WIDTH], OLED_WIDTH);
    }
}

void Display_ShowSpeed(float speed) {
    char buf[16];
    sprintf(buf, "Speed:%.1f cm/s", speed);
    Display_Clear();
    Display_ShowString(0, 0, buf);
}

void Display_ShowValue(const char *label, float value) {
    char buf[32];
    sprintf(buf, "%s:%.2f", label, value);
    Display_ShowString(0, 0, buf);
}