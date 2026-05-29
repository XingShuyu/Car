/**
 * @file    wd35d4.h
 * @brief   WDD35D4角位移传感器驱动接口
 *
 * WDD35D4按电位器型模拟角度传感器使用，滑动端接入PB17，
 * 由SysConfig生成的WDD35D4_ADC采样。驱动负责ADC读取、量程标定、
 * 零点设置、方向设置和角度换算；上层控制逻辑只需要读取角度结果。
 *
 * 硬件配置：
 * - ADC实例：WDD35D4_ADC，由SysConfig生成
 * - 输入引脚：PB17 / ADC1 channel 4
 * - 调用顺序：SYSCFG_DL_init()之后调用WDD35D4_Init()
 */

#ifndef WDD35D4_WD35D4_H_
#define WDD35D4_WD35D4_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** ADC 12位满量程原始值。 */
#define WDD35D4_ADC_MAX_RAW              (4095U)
/** 未标定时使用的最小原始值。 */
#define WDD35D4_DEFAULT_RAW_MIN          (0U)
/** 未标定时使用的最大原始值。 */
#define WDD35D4_DEFAULT_RAW_MAX          (4095U)
/** 标定量程对应的角度范围，单位：deg。 */
#define WDD35D4_ELECTRICAL_ANGLE_DEG     (345.0f)
/** 软件低通滤波关闭值。alpha为0时直接返回本次ADC值。 */
#define WDD35D4_FILTER_DISABLED          (0.0f)
/** 一阶低通滤波默认系数。默认不启用软件低通滤波。 */
#define WDD35D4_DEFAULT_FILTER_ALPHA     (WDD35D4_FILTER_DISABLED)
/** 自动零点校准失败时使用的零点原始值。 */
#define WDD35D4_DEFAULT_ZERO_RAW         (1197U)
/** 自动零点校准默认采样次数。 */
#define WDD35D4_DEFAULT_ZERO_SAMPLES     (32U)

/**
 * @brief WDD35D4一次采样换算后的数据。
 */
typedef struct {
    /** ADC原始值。 */
    uint16_t raw;
    /** 传感器输出电压，单位：V。 */
    float voltage;
    /** 单端角度，范围由raw_min/raw_max映射到0~WDD35D4_ELECTRICAL_ANGLE_DEG。 */
    float angle_deg;
    /** 相对竖直零点的有符号角度，已应用zero_raw和direction。 */
    float signed_angle_deg;
} WDD35D4_Data_t;

/**
 * @brief 初始化WDD35D4驱动状态并使能ADC转换。
 *
 * 必须在SYSCFG_DL_init()之后调用。该函数会重置标定值、零点、方向和滤波状态。
 */
void WDD35D4_Init(void);

/**
 * @brief 设置ADC原始值到角度的有效标定范围。
 *
 * @param raw_min 传感器有效量程低端ADC值。
 * @param raw_max 传感器有效量程高端ADC值，必须大于raw_min。
 *
 * 若raw_max <= raw_min，则本次设置被忽略。
 */
void WDD35D4_SetCalibration(uint16_t raw_min, uint16_t raw_max);

/**
 * @brief 恢复默认标定范围。
 */
void WDD35D4_ResetCalibration(void);

/**
 * @brief 获取当前标定范围。
 *
 * @param raw_min 非NULL时写入当前raw_min。
 * @param raw_max 非NULL时写入当前raw_max。
 */
void WDD35D4_GetCalibration(uint16_t *raw_min, uint16_t *raw_max);

/**
 * @brief 自动采样当前ADC值作为竖直零点。
 *
 * 上电时应保持摆杆在竖直平衡位置，再调用该函数。函数会连续读取
 * sample_count次ADC并取平均值写入zero_raw。
 *
 * @param sample_count 采样次数；传0时使用WDD35D4_DEFAULT_ZERO_SAMPLES。
 * @return true校准成功，false采样失败。
 */
bool WDD35D4_CalibrateZero(uint16_t sample_count);

/**
 * @brief 设置倒立摆竖直平衡点对应的ADC原始值。
 *
 * 后续WDD35D4_ReadSignedAngleDeg()和WDD35D4_RawToSignedAngleDeg()
 * 会以该值为0度。
 *
 * @param zero_raw 竖直平衡点ADC原始值。
 */
void WDD35D4_SetZeroRaw(uint16_t zero_raw);

/**
 * @brief 获取当前有符号角度零点ADC值。
 *
 * @return 当前zero raw。
 */
uint16_t WDD35D4_GetZeroRaw(void);

/**
 * @brief 设置角度正方向。
 *
 * @param direction direction < 0时角度取反，否则保持正向。
 */
void WDD35D4_SetDirection(int8_t direction);

/**
 * @brief 获取当前角度方向。
 *
 * @return 1表示正向，-1表示反向。
 */
int8_t WDD35D4_GetDirection(void);

/**
 * @brief 触发一次ADC转换并读取原始值。
 *
 * @param raw 非NULL时写入ADC原始值。
 * @return true读取成功，false参数无效或ADC转换超时。
 */
bool WDD35D4_ReadRaw(uint16_t *raw);

/**
 * @brief 读取ADC原始值并执行一阶低通滤波。
 *
 * alpha为0时不使用软件低通滤波，直接返回本次ADC原始值。
 * alpha在(0, 1]时使用一阶低通：
 * filtered = filtered * (1 - alpha) + raw * alpha。
 *
 * @param alpha 滤波系数。0表示不滤波；(0,1]表示启用滤波；非法值使用默认值。
 * @param filtered_raw 非NULL时写入处理后的原始值。
 * @return true读取成功，false参数无效或ADC转换失败。
 */
bool WDD35D4_ReadFilteredRaw(float alpha, float *filtered_raw);

/**
 * @brief 读取传感器滑动端电压。
 *
 * @param voltage 非NULL时写入电压值，单位：V。
 * @return true读取成功，false参数无效或ADC转换失败。
 */
bool WDD35D4_ReadVoltage(float *voltage);

/**
 * @brief 读取单端角度。
 *
 * 使用默认滤波设置读取ADC值，并从raw_min~raw_max映射到
 * 0~WDD35D4_ELECTRICAL_ANGLE_DEG。
 *
 * @param angle_deg 非NULL时写入角度，单位：deg。
 * @return true读取成功，false参数无效或ADC转换失败。
 */
bool WDD35D4_ReadAngleDeg(float *angle_deg);

/**
 * @brief 读取相对竖直零点的有符号角度。
 *
 * 该接口用于倒立摆控制。角度 = (raw - zero_raw) / (raw_max - raw_min)
 * * WDD35D4_ELECTRICAL_ANGLE_DEG * direction。
 *
 * @param signed_angle_deg 非NULL时写入有符号角度，单位：deg。
 * @return true读取成功，false参数无效或ADC转换失败。
 */
bool WDD35D4_ReadSignedAngleDeg(float *signed_angle_deg);

/**
 * @brief 读取默认滤波设置下的原始值、电压、单端角和有符号角。
 *
 * @param data 非NULL时写入采样数据。
 * @return true读取成功，false参数无效或ADC转换失败。
 */
bool WDD35D4_ReadData(WDD35D4_Data_t *data);

/**
 * @brief 将ADC原始值换算为电压。
 *
 * @param raw ADC原始值。
 * @return 电压值，单位：V。
 */
float WDD35D4_RawToVoltage(uint16_t raw);

/**
 * @brief 将ADC原始值换算为单端角度。
 *
 * @param raw ADC原始值。
 * @return 单端角度，单位：deg。
 */
float WDD35D4_RawToAngleDeg(uint16_t raw);

/**
 * @brief 将ADC原始值换算为相对竖直零点的有符号角度。
 *
 * @param raw ADC原始值。
 * @return 有符号角度，单位：deg。
 */
float WDD35D4_RawToSignedAngleDeg(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* WDD35D4_WD35D4_H_ */
