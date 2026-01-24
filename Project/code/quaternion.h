#ifndef __QUATERNION_H
#define __QUATERNION_H

#include "zf_common_headfile.h"

#define RAD_TO_DEG  57.295779513f
#define DEG_TO_RAD  0.017453292f
#define MAHONY_KP   0.3f
#define MAHONY_KI   0.002f
#define SAMPLE_FREQ 100.0f

struct Quaternion_t
{
    float q0;
    float q1;
    float q2;
    float q3;
};

struct EulerAngle_t
{
    float roll;
    float pitch;
    float yaw;
};

struct MahonyAHRS_t
{
    float q0;
    float q1;
    float q2;
    float q3;
    float integralFBx;
    float integralFBy;
    float integralFBz;
    uint8 inited;
};

void Mahony_Init(struct MahonyAHRS_t *ahrs);
void Mahony_Update(struct MahonyAHRS_t *ahrs, float gx, float gy, float gz, float ax, float ay, float az, float dt);
void Quaternion_ToEuler(struct Quaternion_t *quat, struct EulerAngle_t *euler);
void Mahony_GetEuler(struct MahonyAHRS_t *ahrs, struct EulerAngle_t *euler);
float Fast_InvSqrt(float x);

#endif
