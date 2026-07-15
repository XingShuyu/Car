#ifndef BUTTON_SELECT_H
#define BUTTON_SELECT_H

#include <stdbool.h>
#include <stdint.h>

bool ButtonSelect_HandleGpioBInterrupt(int gpioB_iidx);
uint8_t ButtonSelect_GetIndex(void);
void ButtonSelect_Reset(void);

#endif
