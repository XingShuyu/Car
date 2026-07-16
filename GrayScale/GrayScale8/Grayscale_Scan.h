#ifndef GRAYSCALE8_SCAN_H_
#define GRAYSCALE8_SCAN_H_

#include "BasicMicroLib/PID.h"
#include "ti_msp_dl_config.h"
#include "grayscale_sensor.h"

//低通滤波系数A,A大，信号更稳但更慢；A小,响应更快但更抖
#define A				0.3

float Grayscale8_Line(PID *pid, bool *sensor_values);
bool Grayscale8_Cross(bool *sensor_values, int status);
void Grayscale8_Zero(bool *sensor_values);
int Grayscale8_OnlineNum(bool *sensor_values);
#endif /* GRAYSCALE8_SCAN_H_ */
