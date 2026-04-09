#ifndef GRAYSCALE_SCAN_H
#define GRAYSCALE_SCAN_H

#include "BasicMicroLib/PID.h"
#include "ti_msp_dl_config.h"
#include "grayscale_sensor.h"

//低通滤波系数A,A大，信号更稳但更慢；A小,响应更快但更抖
#define A				0.3

float Grayscale_Line(PID *pid,bool *sensor_values);
bool Grayscale_Cross(bool *sensor_values, int status);
void Grayscale_Zero(bool *sensor_values);

#endif