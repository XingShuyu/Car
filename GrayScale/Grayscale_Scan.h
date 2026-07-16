/**
 * @file Grayscale_Scan.h
 * @brief 8/12 路灰度循迹、路口与原地转弯的统一接口。
 */

#ifndef GRAYSCALE_SCAN_H_
#define GRAYSCALE_SCAN_H_

#include "BasicMicroLib/PID.h"
#include "GrayScale/grayscale_sensor.h"
#include <stdbool.h>

typedef enum {
    GrayscaleTurnRight = 0,
    GrayscaleTurnLeft,
} Grayscale_TurnDirection_t;

/**
 * 返回当前模块的循迹转向修正量。
 *
 * 参数 pid 仅保留旧八路调用兼容性；八路沿用其原有内部 PID，十二路使用本
 * 模块持有的独立 PID，不与传入参数联动。
 */
float Grayscale_Line(PID *pid,
                     bool sensor_values[GRAYSCALE_SENSOR_CHANNELS]);

/** 判断十字/直角：status=0 十字，1 右直角，2 左直角。 */
bool Grayscale_Cross(bool sensor_values[GRAYSCALE_SENSOR_CHANNELS],
                     int status);

/** 清空当前模块的循迹状态，并填入统一数组的中心线状态。 */
void Grayscale_Zero(bool sensor_values[GRAYSCALE_SENSOR_CHANNELS]);

/** 读取并返回当前有效通道数。 */
int Grayscale_OnlineNum(bool sensor_values[GRAYSCALE_SENSOR_CHANNELS]);

/**
 * 一次读取完成原地转弯判断。
 *
 * reached 为 true 时应停止转动；speedScale 为 [0.1, 1.0] 的建议倍率。
 * 返回 false 表示当前读取失败。
 */
bool Grayscale_GetTurnControl(Grayscale_TurnDirection_t direction,
                              bool *reached, float *speedScale);

/** 设置十二路独立 PID，设置后会清空十二路积分和微分历史。 */
bool Grayscale_Set12Pid(const PID *pid);

/** 获取十二路独立 PID 参数，返回的指针可用于查看当前参数。 */
const PID *Grayscale_Get12Pid(void);

/** 设置十二路最终 irr 适配倍率；非零值均有效，可用于校正转向方向。 */
bool Grayscale_Set12IrrScale(float scale);

/** 获取十二路最终 irr 适配倍率。 */
float Grayscale_Get12IrrScale(void);

#endif /* GRAYSCALE_SCAN_H_ */
