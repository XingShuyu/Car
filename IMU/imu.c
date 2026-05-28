/**
 * @file    imu.c
 * @brief  IMU for TI MSPM0G3507.
 */
#include "IMU/imu.h"
#include "BasicMicroLib/getTime.h"

enum IMUDevice imuDevice;
float yaw;
uint32_t lastTime = 0;

int IMU_Init(void) {
	// try JY901S
	if (JY901S_Init()) {
		imuDevice = JY901S;
		JY901S_ZeroYaw();
		return 1;
	} else if (MPU6050_Init()) {
		imuDevice = MPU6050;
		MPU6050_SetFilter(0.35f, 0.1f, 0.01f);
		MPU6050_CalibrateGyro(20);
		return 2;
	} else {
		return 0;
	}
}

bool IMU_ReadAll(IMU_Data_t *IMU_Data) {
	if (imuDevice == JY901S) {
        JY901S_ReadAll(IMU_Data);
	} else {
        MPU6050_ReadAllCalibrated(IMU_Data);
        if(yaw == 1145){
            yaw = 0.0;
            lastTime = getNowMs();
            return true;
        }
        yaw += IMU_Data->gz*getTimeMs(getNowMs(), lastTime)/1000.0;
        lastTime = getNowMs();
        if(yaw>180.0){
            yaw-=360;
        }
        else if (yaw<=-180.0) {
            yaw+=360;
        }
        IMU_Data->yaw = yaw;
	}
}

void IMU_ZeroYaw(void) {
	if (imuDevice == JY901S) {
        JY901S_ZeroYaw();
	} else {
        yaw = 1145;
	}
}