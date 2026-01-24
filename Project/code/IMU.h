#ifndef _IMU_H_
#define _IMU_H_

#include "zf_common_headfile.h"
#include "quaternion.h"

#define IMU_SAMPLE_RATE 100
#define DT (1.0f / IMU_SAMPLE_RATE)

struct IMUData
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float q0;
    float q1;
    float q2;
    float q3;
    float roll;
    float pitch;
    float yaw;
};

extern struct IMUData imu;
extern struct MahonyAHRS_t ahrs;

uint8 imu_init(void);
void imu_update(void);
void imu_get_data(void);
float angle_normalize(float angle);

#endif
