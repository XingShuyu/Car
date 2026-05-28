/**
 * @file    imu_data.h
 * @brief   Shared IMU data layout used by all IMU drivers.
 */

#ifndef IMU_DATA_H_
#define IMU_DATA_H_

typedef struct {
    float ax;      /* Acceleration X [g] */
    float ay;      /* Acceleration Y [g] */
    float az;      /* Acceleration Z [g] */
    float gx;      /* Angular velocity X [deg/s] */
    float gy;      /* Angular velocity Y [deg/s] */
    float gz;      /* Angular velocity Z [deg/s] */
    float hx;      /* Magnetic field X, raw unit */
    float hy;      /* Magnetic field Y, raw unit */
    float hz;      /* Magnetic field Z, raw unit */
    float roll;    /* Roll angle [deg] */
    float pitch;   /* Pitch angle [deg] */
    float yaw;     /* Yaw angle [deg] */
    float temp;    /* Temperature [deg C] */
} JY901S_Data_t;

typedef JY901S_Data_t IMU_Data_t;
/* Backward-compatible alias for old MPU6050 call sites. Prefer JY901S_Data_t
 * or IMU_Data_t in shared application code. */
typedef JY901S_Data_t MPU6050_Data_t;


#endif /* IMU_DATA_H_ */
