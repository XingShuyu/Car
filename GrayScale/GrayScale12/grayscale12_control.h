/**
 * @file grayscale12_control.h
 * @brief NCHD12 归一化循迹与 PID 转向控制。
 *
 * 本控制层与 I2C 设备层分离：归一化位置只依赖 raw12，因此可以在没有硬件时
 * 使用 Grayscale12_LineFromRaw() 调试 PID；Grayscale12_Line() 则完成读取、
 * 归一化和 PID 计算，并直接返回电机转向修正量。
 */

#ifndef GRAYSCALE12_CONTROL_H_
#define GRAYSCALE12_CONTROL_H_

#include "BasicMicroLib/PID.h"
#include "GrayScale/GrayScale12/grayscale12.h"
#include <stdbool.h>

/**
 * 十二路循迹控制器状态。
 *
 * pid 指向可由上层实时调参的 PID 参数。pid->t 的单位为毫秒；当 t 为 0 时，
 * 控制器使用 10 ms 作为默认周期。pid->i_Max 为积分项（不是积分累计值）的
 * 绝对限幅，设置为 0 或负数时不限制积分项。
 */
typedef struct {
    PID *pid;
    float integral;
    float previousError;
    float lastPosition;
    float lastOutput;
    bool hasPreviousError;
    bool hasPosition;
} Grayscale12_LineController_t;

/** 绑定 PID 参数并清空循迹控制状态。pid 不能为空。 */
bool Grayscale12_LineController_Init(Grayscale12_LineController_t *controller,
                                     PID *pid);

/** 清空积分、微分历史和丢线保持值。 */
void Grayscale12_LineController_Reset(Grayscale12_LineController_t *controller);

/**
 * 将低 12 位灰度状态按重心法归一化到 [-1.0, +1.0]。
 *
 * bit1（最右侧）为 -1，bit12（最左侧）为 +1。没有任何有效通道时返回 false。
 */
bool Grayscale12_NormalizeRaw(uint16_t raw12, float *position);

/**
 * 对已读取的 raw12 进行归一化和 PID 计算，返回转向修正量。
 *
 * lineDetected 可为 NULL。丢线时不会继续积分或计算微分，而是保持上一次
 * 有效控制输出；从未检测到线时返回 0。
 */
float Grayscale12_LineFromRaw(Grayscale12_LineController_t *controller,
                              uint16_t raw12, bool *lineDetected);

/**
 * 读取 NCHD12、填充 sensorValues 并返回 PID 转向修正量。
 *
 * sensorValues 可为 NULL；若 I2C 通信失败，lineDetected 会被置为 false，
 * 函数保持最近一次有效输出。
 */
float Grayscale12_Line(Grayscale12_LineController_t *controller,
                       bool sensorValues[GRAYSCALE12_CHANNELS],
                       bool *lineDetected);

#endif /* GRAYSCALE12_CONTROL_H_ */
