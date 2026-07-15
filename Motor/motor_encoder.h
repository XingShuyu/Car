#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

bool MotorEncoder_HandleGpioAInterrupt(int gpioA_iidx);
bool MotorEncoder_HandleGpioBInterrupt(int gpioB_iidx);
void MotorEncoder_ReadAndClear(int32_t *left_delta_counts,
							   int32_t *right_delta_counts);
void MotorEncoder_ResetCounts(void);

#endif
