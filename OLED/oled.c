/**
 * @file oled.c
 * @brief SSD1306 OLED low-level driver for MSPM0 DriverLib I2C.
 */

#include "oled.h"
#include "BasicMicroLib/i2c_display_bus.h"
#include "ti_msp_dl_config.h"
#include <stdlib.h>
#include <string.h>

#define OLED_CONTROL_CMD    0x00u
#define OLED_CONTROL_DATA   0x40u
#define OLED_I2C_FIFO_SIZE  8u
#define OLED_DATA_CHUNK_SIZE (OLED_I2C_FIFO_SIZE - 1u)

static uint8_t s_framebuffer[OLED_BUFFER_SIZE];
static uint8_t s_i2cAddress = 0x3Cu;
static bool s_isReady = false;

static bool write_command(uint8_t command)
{
    uint8_t buffer[2] = {OLED_CONTROL_CMD, command};
    return I2C_DisplayBus_Write(s_i2cAddress, buffer, sizeof(buffer));
}

static bool write_data(const uint8_t *data, uint8_t length)
{
    uint8_t buffer[OLED_DATA_CHUNK_SIZE + 1u];

    if ((data == NULL) || (length == 0u) || (length > OLED_DATA_CHUNK_SIZE)) {
        return false;
    }

    buffer[0] = OLED_CONTROL_DATA;
    memcpy(&buffer[1], data, length);
    return I2C_DisplayBus_Write(s_i2cAddress, buffer, (uint8_t)(length + 1u));
}

static bool set_page_address(uint8_t page, uint8_t column)
{
    if ((page >= OLED_PAGE_COUNT) || (column >= OLED_WIDTH)) {
        return false;
    }

    return write_command((uint8_t)(0xB0u | page)) &&
           write_command((uint8_t)(0x00u | (column & 0x0Fu))) &&
           write_command((uint8_t)(0x10u | ((column >> 4u) & 0x0Fu)));
}

static void set_pixel_i(int x, int y, bool on)
{
    if ((x < 0) || (x >= (int)OLED_WIDTH) ||
        (y < 0) || (y >= (int)OLED_HEIGHT)) {
        return;
    }

    OLED_SetPixel((uint8_t)x, (uint8_t)y, on);
}

static bool probe_address(uint8_t address)
{
    uint8_t buffer[2] = {OLED_CONTROL_CMD, 0xAEu};
    return I2C_DisplayBus_Write(address, buffer, sizeof(buffer));
}

static bool select_oled_address(void)
{
    static const uint8_t addresses[] = {0x3Cu, 0x3Du};

    for (uint8_t i = 0u; i < sizeof(addresses); i++) {
        if (probe_address(addresses[i])) {
            s_i2cAddress = addresses[i];
            return true;
        }
    }

    return false;
}

static bool send_init_sequence(void)
{
    static const uint8_t initCommands[] = {
        0xAEu,       /* display off */
        0xD5u, 0x80u, /* clock divide */
        0xA8u, 0x3Fu, /* multiplex ratio */
        0xD3u, 0x00u, /* display offset */
        0x40u,       /* display start line */
        0x8Du, 0x14u, /* charge pump on */
        0x20u, 0x02u, /* page addressing */
        0xA1u,       /* segment remap */
        0xC8u,       /* COM scan direction */
        0xDAu, 0x12u, /* COM pins */
        0x81u, 0xCFu, /* contrast */
        0xD9u, 0xF1u, /* pre-charge */
        0xDBu, 0x40u, /* VCOMH */
        0xA4u,       /* display follows RAM */
        0xA6u,       /* normal display */
        0xAFu        /* display on */
    };

    for (uint8_t i = 0u; i < sizeof(initCommands); i++) {
        if (!write_command(initCommands[i])) {
            return false;
        }
    }

    OLED_Clear();
    return OLED_Update();
}

bool OLED_Init(void)
{
    s_isReady = false;

    for (volatile uint32_t i = 0u; i < 10000u; i++) {
        __NOP();
    }

    if (!select_oled_address()) {
        return false;
    }

    s_isReady = send_init_sequence();
    return s_isReady;
}

bool OLED_IsReady(void)
{
    return s_isReady;
}

void OLED_Clear(void)
{
    memset(s_framebuffer, 0x00, sizeof(s_framebuffer));
}

bool OLED_UpdatePage(uint8_t page)
{
    uint8_t column = 0u;

    if (page >= OLED_PAGE_COUNT) {
        return false;
    }

    while (column < OLED_WIDTH) {
        uint8_t chunk = (uint8_t)(OLED_WIDTH - column);
        if (chunk > OLED_DATA_CHUNK_SIZE) {
            chunk = OLED_DATA_CHUNK_SIZE;
        }

        if (!set_page_address(page, column)) {
            return false;
        }
        if (!write_data(&s_framebuffer[(page * OLED_WIDTH) + column], chunk)) {
            return false;
        }
        column = (uint8_t)(column + chunk);
    }

    return true;
}

bool OLED_Update(void)
{
    for (uint8_t page = 0u; page < OLED_PAGE_COUNT; page++) {
        if (!OLED_UpdatePage(page)) {
            return false;
        }
    }

    return true;
}

void OLED_SetPixel(uint8_t x, uint8_t y, bool on)
{
    uint16_t index;
    uint8_t mask;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT)) {
        return;
    }

    index = (uint16_t)x + (uint16_t)((y / 8u) * OLED_WIDTH);
    mask = (uint8_t)(1u << (y % 8u));

    if (on) {
        s_framebuffer[index] |= mask;
    } else {
        s_framebuffer[index] &= (uint8_t)~mask;
    }
}

void OLED_DrawPoint(uint8_t x, uint8_t y)
{
    OLED_SetPixel(x, y, true);
}

void OLED_ClearPoint(uint8_t x, uint8_t y)
{
    OLED_SetPixel(x, y, false);
}

void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    int dx = abs((int)x2 - (int)x1);
    int sx = (x1 < x2) ? 1 : -1;
    int dy = -abs((int)y2 - (int)y1);
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx + dy;

    while (true) {
        OLED_DrawPoint(x1, y1);
        if ((x1 == x2) && (y1 == y2)) {
            break;
        }

        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x1 = (uint8_t)((int)x1 + sx);
        }
        if (e2 <= dx) {
            err += dx;
            y1 = (uint8_t)((int)y1 + sy);
        }
    }
}

void OLED_DrawCircle(uint8_t x0, uint8_t y0, uint8_t r)
{
    int x = 0;
    int y = r;
    int err = 1 - (int)r;
    int cx = x0;
    int cy = y0;

    while (x <= y) {
        set_pixel_i(cx + x, cy + y, true);
        set_pixel_i(cx - x, cy + y, true);
        set_pixel_i(cx + x, cy - y, true);
        set_pixel_i(cx - x, cy - y, true);
        set_pixel_i(cx + y, cy + x, true);
        set_pixel_i(cx - y, cy + x, true);
        set_pixel_i(cx + y, cy - x, true);
        set_pixel_i(cx - y, cy - x, true);

        x++;
        if (err < 0) {
            err += (2 * x) + 1;
        } else {
            y--;
            err += (2 * (x - y)) + 1;
        }
    }
}

uint8_t *OLED_GetBuffer(void)
{
    return s_framebuffer;
}
