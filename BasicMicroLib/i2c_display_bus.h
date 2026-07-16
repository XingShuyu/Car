/**
 * @file i2c_display_bus.h
 * @brief I2C1（OLED 显示接口）共享总线的同步访问接口。
 *
 * OLED 与 NCHD12 十二路灰度模块挂在同一条 I2C1 总线上。本模块只负责
 * 单次、阻塞式传输；调用方应在 SYSCFG_DL_init() 之后、主循环上下文中使用，
 * 不要从中断服务程序发起传输。
 */

#ifndef I2C_DISPLAY_BUS_H_
#define I2C_DISPLAY_BUS_H_

#include <stdbool.h>
#include <stdint.h>

/** 检测指定 7 位 I2C 从机地址是否应答。 */
bool I2C_DisplayBus_Probe(uint8_t address7bit);

/**
 * 向指定 7 位地址写入数据。
 *
 * 单次发送最大 8 字节，与 MSPM0 I2C 控制器 TX FIFO 大小一致。
 */
bool I2C_DisplayBus_Write(uint8_t address7bit, const uint8_t *data,
                          uint8_t length);

/**
 * 先写入数据，随后通过 repeated START 读取数据。
 *
 * 该接口适用于 PCA9555 等“写寄存器地址后立即读”的设备。txLength 最大
 * 为 8 字节，地址参数始终使用 7 位格式。
 */
bool I2C_DisplayBus_WriteRead(uint8_t address7bit, const uint8_t *txData,
                              uint8_t txLength, uint8_t *rxData,
                              uint8_t rxLength);

#endif /* I2C_DISPLAY_BUS_H_ */
