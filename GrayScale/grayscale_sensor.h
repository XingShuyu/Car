/**
 * @file grayscale_sensor.h
 * @brief 8/12 路灰度模块的统一传感器访问接口。
 *
 * 统一数组固定按 P1（右侧）到 P12（左侧）排列。8 路回退模式将旧模块的
 * 8 路数据放在 P3 至 P10，P1、P2、P11、P12 固定为 0。
 */

#ifndef GRAYSCALE_SENSOR_H_
#define GRAYSCALE_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>

#define GRAYSCALE_SENSOR_CHANNELS (12u)

typedef enum {
    GrayscaleDriver8 = 0,
    GrayscaleDriver12,
} Grayscale_Driver_t;

/**
 * 优先初始化 I2C 十二路模块，失败时自动选择八路模块。
 *
 * 选择仅在调用本函数时进行；运行中 I2C 读失败不会切换后端。
 */
bool Grayscale_Init(void);

/** 当前固定选中的灰度模块。 */
Grayscale_Driver_t Grayscale_GetActiveDriver(void);

/** 最近一次统一读取是否成功。八路 GPIO 后端始终返回 true。 */
bool Grayscale_LastReadOk(void);

/** 与旧工程兼容的初始化入口，等价于 Grayscale_Init()。 */
void Grayscale_Sensor_Init(void);

/** 读取统一的 P1 至 P12 状态。读取失败时所有输出通道均置为 false。 */
void Grayscale_Sensor_Read_All(bool sensor_values[GRAYSCALE_SENSOR_CHANNELS]);

/** 读取统一数组中间四路 P5 至 P8，其余元素保持调用前的值。 */
void Grayscale_Sensor_Read_Main(bool sensor_values[GRAYSCALE_SENSOR_CHANNELS]);

/** 读取统一数组两侧八路 P1 至 P4、P9 至 P12，其余元素保持调用前的值。 */
void Grayscale_Sensor_Read_Other(bool sensor_values[GRAYSCALE_SENSOR_CHANNELS]);

/** 读取统一索引 0 至 11 的单通道状态；无效索引或读取失败返回 false。 */
bool Grayscale_Sensor_Read_Single(uint8_t channel);

/** 旧代码兼容别名，读取统一索引 0 至 11。 */
bool _read_channel_stable(uint8_t channel);

#endif /* GRAYSCALE_SENSOR_H_ */
