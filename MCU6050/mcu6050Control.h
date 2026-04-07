#ifndef MPU6050CONTROL_H_
#define MPU6050CONTROL_H_

float Angle_PID_Calculate(PID *AngelPid, float targetAngle, float nowAngle);

#endif