/**
 * @file    imu.h
 * @brief   Unified IMU include entry.
 */

#ifndef IMU_H_
#define IMU_H_

#include "imu_data.h"
#include "HWT101/hwt101.h"
#include "JY901S/jy901s.h"
#include "MPU6050/mpu6050.h"

enum IMUDevice {
	HWT101 = 1,
	JY901S = 2,
	MPU6050 = 3
};

int IMU_Init(void);
bool IMU_ReadAll(IMU_Data_t* IMU_Data);
void IMU_ZeroYaw(void);


#endif /* IMU_H_ */
