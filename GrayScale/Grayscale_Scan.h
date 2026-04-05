#ifndef GRAYSCALE_SCAN_H
#define GRAYSCALE_SCAN_H

#include "BasicMicroLib/PID.h"
#include "grayscale_sensor.h"

//低通滤波系数A,A大，信号更稳但更慢；A小,响应更快但更抖
#define A				0.5

float Grayscale_Line(bool *sensor_values, PID *pid);
bool Grayscale_Cross(bool *sensor_values, int status);

#endif