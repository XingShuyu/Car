/**
 * @file    imu_data.h
 * @brief   Shared IMU data layout used by all IMU drivers.
 */

#ifndef IMU_DATA_H_
#define IMU_DATA_H_

#include <stdint.h>

#define IMU_DEVICE_MASK_HWT101  (0x01u)
#define IMU_DEVICE_MASK_JY901S  (0x02u)
#define IMU_DEVICE_MASK_MPU6050 (0x04u)

#define IMU_VALID_AX    (1u << 0)
#define IMU_VALID_AY    (1u << 1)
#define IMU_VALID_AZ    (1u << 2)
#define IMU_VALID_GX    (1u << 3)
#define IMU_VALID_GY    (1u << 4)
#define IMU_VALID_GZ    (1u << 5)
#define IMU_VALID_HX    (1u << 6)
#define IMU_VALID_HY    (1u << 7)
#define IMU_VALID_HZ    (1u << 8)
#define IMU_VALID_ROLL  (1u << 9)
#define IMU_VALID_PITCH (1u << 10)
#define IMU_VALID_YAW   (1u << 11)
#define IMU_VALID_TEMP  (1u << 12)

#define IMU_VALID_ACCEL (IMU_VALID_AX | IMU_VALID_AY | IMU_VALID_AZ)
#define IMU_VALID_GYRO  (IMU_VALID_GX | IMU_VALID_GY | IMU_VALID_GZ)
#define IMU_VALID_MAG   (IMU_VALID_HX | IMU_VALID_HY | IMU_VALID_HZ)
#define IMU_VALID_ANGLE (IMU_VALID_ROLL | IMU_VALID_PITCH | IMU_VALID_YAW)

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
    uint32_t validMask; /* Bitmask of fields populated in this sample. */
    uint8_t sourceMask; /* IMU_DEVICE_MASK_* contributors for this sample. */
} JY901S_Data_t;

typedef JY901S_Data_t IMU_Data_t;
/* Backward-compatible alias for old MPU6050 call sites. Prefer JY901S_Data_t
 * or IMU_Data_t in shared application code. */
typedef JY901S_Data_t MPU6050_Data_t;


#endif /* IMU_DATA_H_ */
