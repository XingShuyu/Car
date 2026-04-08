#include "BasicMicroLib/PID.h"
#include "mpu6050.h"

float Angle_PID_Calculate(PID *AngelPid, float targetAngle, float nowAngle) {
	static float oldError, result;
	float error = targetAngle - nowAngle;
	result += error-oldError;
	oldError = error;
	return result/90.0;
}