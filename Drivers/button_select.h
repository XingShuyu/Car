#ifndef BUTTON_SELECT_H
#define BUTTON_SELECT_H

#include <stdbool.h>
#include <stdint.h>

/** 启动菜单中可消费的按键事件。 */
typedef enum ButtonSelect_Event {
	ButtonSelectEventNone = 0,
	ButtonSelectEventNext,
	ButtonSelectEventStart,
} ButtonSelect_Event;

/**
 * @brief 处理 GPIOB 按键中断。
 *
 * PB23 产生 Next 事件，PB26 产生 Start 事件。函数仅记录事件，
 * 菜单状态和路线选择由主循环负责。
 */
bool ButtonSelect_HandleGpioBInterrupt(int gpioB_iidx);

/**
 * @brief 读取并清除一个待处理按键事件。
 * @return ButtonSelectEventNone 表示当前没有事件。
 */
ButtonSelect_Event ButtonSelect_TakeEvent(void);

/** 清除开机初始化期间遗留的按键事件，并重新开始防抖计时。 */
void ButtonSelect_ResetEvents(void);

#endif
