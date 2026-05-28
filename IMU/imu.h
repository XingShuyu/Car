/**
 * @file    imu.h
 * @brief   Unified IMU include entry.
 */

#ifndef IMU_H_
#define IMU_H_

#include "imu_data.h"
#include "JY901S/jy901s.h"
#include "MPU6050/mpu6050.h"

 enum IMUDevice{
    JY901S = 1,
    MPU6050 = 2
 };

int IMU_Init(void);
bool IMU_ReadAll(IMU_Data_t* IMU_Data);
void IMU_ZeroYaw(void);


#endif /* IMU_H_ */
