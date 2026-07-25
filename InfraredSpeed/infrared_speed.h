/**
 * @file infrared_speed.h
 * @brief 两个对射式红外测速门的底层驱动。
 *
 * 首个被小球遮挡的传感器启动计时，另一个传感器被遮挡时停止计时。
 * PB22 为 Gate A，PB01 为 Gate B；两门间距离为 0.30 m。
 */

#ifndef INFRARED_SPEED_H
#define INFRARED_SPEED_H

#include <stdbool.h>
#include <stdint.h>

/* 两个传感器中心间距。修改实际安装间距时只需修改此处。 */
#define INFRARED_SPEED_GATE_DISTANCE_MM 300U

/*
 * 常开 NPN 输出在遮挡时会下拉到低电平，故默认值为 1。
 * 若所用传感器在遮挡时输出高电平（例如常开 PNP），改为 0。
 * SysConfig 已配置双边沿中断，修改此宏后无需再修改中断边沿。
 */
#define INFRARED_SPEED_ACTIVE_LOW 1U

/* 首次遮挡后未经过另一个门的最大允许时间。 */
#define INFRARED_SPEED_TIMEOUT_US 10000000UL

typedef enum InfraredSpeed_Gate {
	InfraredSpeedGateA = 0,
	InfraredSpeedGateB,
} InfraredSpeed_Gate;

typedef enum InfraredSpeed_Status {
	InfraredSpeedStatusArmed = 0,
	InfraredSpeedStatusWaitingSecondGate,
	InfraredSpeedStatusComplete,
	InfraredSpeedStatusTimeout,
} InfraredSpeed_Status;

typedef struct InfraredSpeed_Result {
	InfraredSpeed_Gate firstGate;
	InfraredSpeed_Gate secondGate;
	uint32_t elapsedUs;
	float speedMps;
} InfraredSpeed_Result;

/** 初始化驱动状态；必须在 TimeBase_Init() 后调用。 */
void InfraredSpeed_Init(void);

/** 丢弃上一轮结果并等待任意一个测速门先被遮挡。 */
void InfraredSpeed_Rearm(void);

/**
 * @brief 推进超时判断。
 * @param nowUs 当前微秒时间戳，传入 getNowUs() 的返回值。
 */
void InfraredSpeed_Update(uint32_t nowUs);

/** @brief 供 GROUP1_IRQHandler 分发 GPIOB 中断。 */
bool InfraredSpeed_HandleGpioBInterrupt(int gpioB_iidx);

InfraredSpeed_Status InfraredSpeed_GetStatus(void);

/** @return 当前轮次中先触发的测速门；仅在等待或完成状态下有意义。 */
InfraredSpeed_Gate InfraredSpeed_GetFirstGate(void);

/**
 * @brief 取得一次完成的测速结果。
 * @return 仅当状态为 InfraredSpeedStatusComplete 时返回 true。
 */
bool InfraredSpeed_GetResult(InfraredSpeed_Result *result);

#endif /* INFRARED_SPEED_H */
