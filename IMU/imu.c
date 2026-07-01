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
static uint8_t imuReadyMask = 0u;

int IMU_Init(void) {
	imuReadyMask = 0u;
	imuDevice = 0;
	mpuYawReady = false;

	if (HWT101_Init()) {
		imuReadyMask |= IMU_DEVICE_MASK_HWT101;
		imuDevice = HWT101;
		HWT101_ZeroYaw();
	} else if (JY901S_Init()) {
		imuReadyMask |= IMU_DEVICE_MASK_JY901S;
		imuDevice = JY901S;
		JY901S_ZeroYaw();
	}

	if (MPU6050_Init()) {
		imuReadyMask |= IMU_DEVICE_MASK_MPU6050;
		imuDevice = MPU6050;
		MPU6050_SetFilter(0.35f, 0.1f, 0.01f);
		MPU6050_CalibrateGyro(20);
		yaw = 0.0f;
		lastTime = getNowMs();
		mpuYawReady = true;
	}

	if ((imuReadyMask & IMU_DEVICE_MASK_HWT101) != 0u) {
		imuDevice = HWT101;
	} else if ((imuReadyMask & IMU_DEVICE_MASK_JY901S) != 0u) {
		imuDevice = JY901S;
	} else if ((imuReadyMask & IMU_DEVICE_MASK_MPU6050) != 0u) {
		imuDevice = MPU6050;
	}

	return imuReadyMask;
}

uint8_t IMU_GetReadyMask(void) {
	return imuReadyMask;
}

bool IMU_IsDeviceReady(enum IMUDevice device) {
	switch (device) {
	case HWT101:
		return (imuReadyMask & IMU_DEVICE_MASK_HWT101) != 0u;
	case JY901S:
		return (imuReadyMask & IMU_DEVICE_MASK_JY901S) != 0u;
	case MPU6050:
		return (imuReadyMask & IMU_DEVICE_MASK_MPU6050) != 0u;
	default:
		return false;
	}
}

static bool read_mpu6050(IMU_Data_t *out) {
	uint32_t nowTime;

	if (!MPU6050_ReadAllCalibrated(out)) {
		return false;
	}

	nowTime = getNowMs();
	if (!mpuYawReady) {
		yaw = 0.0f;
		lastTime = nowTime;
		mpuYawReady = true;
		out->yaw = yaw;
		out->validMask |= IMU_VALID_YAW;
		out->sourceMask |= IMU_DEVICE_MASK_MPU6050;
		return true;
	}

	yaw += out->gz * getTimeMs(nowTime, lastTime) / 1000.0f;
	lastTime = nowTime;
	if (yaw > 180.0f) {
		yaw -= 360.0f;
	} else if (yaw <= -180.0f) {
		yaw += 360.0f;
	}
	out->yaw = yaw;
	out->validMask |= IMU_VALID_YAW;
	out->sourceMask |= IMU_DEVICE_MASK_MPU6050;
	return true;
}

static void fill_from_mpu6050(IMU_Data_t *dst, const IMU_Data_t *src) {
	dst->ax = src->ax;
	dst->ay = src->ay;
	dst->az = src->az;
	dst->gx = src->gx;
	dst->gy = src->gy;
	dst->gz = src->gz;
	dst->roll = src->roll;
	dst->pitch = src->pitch;
	dst->yaw = src->yaw;
	dst->temp = src->temp;
	dst->validMask |= src->validMask;
	dst->sourceMask |= src->sourceMask;
}

static void fill_from_jy901s(IMU_Data_t *dst, const IMU_Data_t *src) {
	uint8_t sourceMask = dst->sourceMask | src->sourceMask;

	*dst = *src;
	dst->sourceMask = sourceMask;
}

static void fill_from_hwt101(IMU_Data_t *dst, const IMU_Data_t *src) {
	const uint32_t hwtFields = IMU_VALID_GY | IMU_VALID_GZ | IMU_VALID_YAW;

	dst->gy = src->gy;
	dst->gz = src->gz;
	dst->yaw = src->yaw;
	dst->validMask = (dst->validMask & ~hwtFields) |
					 (src->validMask & hwtFields);
	dst->sourceMask |= src->sourceMask;
}

bool IMU_ReadAll(IMU_Data_t *IMU_Data) {
	IMU_Data_t merged = {0};
	IMU_Data_t sample = {0};
	bool hasData = false;

	if (IMU_Data == NULL) {
		return false;
	}

	if (((imuReadyMask & IMU_DEVICE_MASK_MPU6050) != 0u) &&
		read_mpu6050(&sample)) {
		fill_from_mpu6050(&merged, &sample);
		hasData = true;
	}

	if (((imuReadyMask & IMU_DEVICE_MASK_JY901S) != 0u) &&
		JY901S_ReadAll(&sample)) {
		fill_from_jy901s(&merged, &sample);
		hasData = true;
	}

	if (((imuReadyMask & IMU_DEVICE_MASK_HWT101) != 0u) &&
		HWT101_ReadAll(&sample)) {
		fill_from_hwt101(&merged, &sample);
		hasData = true;
	}

	if (!hasData) {
		return false;
	}

	*IMU_Data = merged;
	return true;
}

void IMU_ZeroYaw(void) {
	if ((imuReadyMask & IMU_DEVICE_MASK_HWT101) != 0u) {
		(void)HWT101_ZeroYaw();
	}
	if ((imuReadyMask & IMU_DEVICE_MASK_JY901S) != 0u) {
		(void)JY901S_ZeroYaw();
	}
	if ((imuReadyMask & IMU_DEVICE_MASK_MPU6050) != 0u) {
		yaw = 0.0f;
		lastTime = getNowMs();
		mpuYawReady = true;
	}
}
