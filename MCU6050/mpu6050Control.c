#include "BasicMicroLib/PID.h"
#include "mpu6050.h"

float Angle_PID_Calculate(PID *AngelPid, float targetAngle, float nowAngle) {
	static float oldError, prevError, result;
	float error = targetAngle - nowAngle;
	result += PID_calculate(AngelPid, error, oldError, prevError);
	prevError = oldError;
	oldError = error;
	printf("Back:%.2f\r\n",result);
	return result/90.0;
}