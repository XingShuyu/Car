#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

void Buzzer_Beep(void);
/** 启动一次 100 ms 单响后立即返回，不阻塞主循环。 */
void Buzzer_BeepOnceAsync(uint32_t nowMs);
/** 主循环周期调用，在单响时间到达后关闭蜂鸣器。 */
void Buzzer_Update(uint32_t nowMs);

#endif
