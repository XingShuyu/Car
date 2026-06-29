/**
 * @file    imu.c
 * @brief  IMU for TI MSPM0G3507.
 */
#include "IMU/imu.h"
#include "BasicMicroLib/getTime.h"

enum IMUDevice imuDevice;
float yaw;
uint32_t lastTime = 0;
static bool mpuYawReady = false;

int IMU_Init(void) {
	if (HWT101_Init()) {
		imuDevice = HWT101;
		HWT101_ZeroYaw();
		return 1;
	} else if (JY901S_Init()) {
		imuDevice = JY901S;
		JY901S_ZeroYaw();
		return 2;
	} else if (MPU6050_Init()) {
		imuDevice = MPU6050;
		MPU6050_SetFilter(0.35f, 0.1f, 0.01f);
		MPU6050_CalibrateGyro(20);
		yaw = 0.0f;
		lastTime = getNowMs();
		mpuYawReady = true;
		return 3;
	} else {
		return 0;
	}
}

bool IMU_ReadAll(IMU_Data_t *IMU_Data) {
	uint32_t nowTime;

	if (IMU_Data == NULL) {
		return false;
	}

	if (imuDevice == HWT101) {
		return HWT101_ReadAll(IMU_Data);
	}
	if (imuDevice == JY901S) {
		return JY901S_ReadAll(IMU_Data);
	}
	if (imuDevice != MPU6050) {
		return false;
	}

	if (!MPU6050_ReadAllCalibrated(IMU_Data)) {
		return false;
	}

	nowTime = getNowMs();
	if (!mpuYawReady) {
		yaw = 0.0f;
		lastTime = nowTime;
		mpuYawReady = true;
		IMU_Data->yaw = yaw;
		return true;
	}

	yaw += IMU_Data->gz * getTimeMs(nowTime, lastTime) / 1000.0f;
	lastTime = nowTime;
	if (yaw > 180.0f) {
		yaw -= 360.0f;
	} else if (yaw <= -180.0f) {
		yaw += 360.0f;
	}
	IMU_Data->yaw = yaw;
	return true;
}

void IMU_ZeroYaw(void) {
	if (imuDevice == HWT101) {
		HWT101_ZeroYaw();
	} else if (imuDevice == JY901S) {
		JY901S_ZeroYaw();
	} else {
		yaw = 0.0f;
		lastTime = getNowMs();
		mpuYawReady = true;
	}
}
