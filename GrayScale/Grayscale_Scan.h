#ifndef GRAYSCALE_SCAN_H
#define GRAYSCALE_SCAN_H

#include "BasicMicroLib/PID.h"
#include "grayscale_sensor.h"

float Grayscale_Line(uint16_t *sensor_values, PID *pid);
bool Grayscale_Cross(uint16_t *sensor_values, uint16_t threshold,
					 int status);

#endif