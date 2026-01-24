#include "quaternion.h"

float my_sqrt(x)
float x;
{
    float guess;
    float epsilon;
    uint16 i;

    guess = x;
    epsilon = 0.00001f;

    for(i = 0; i < 20; i++)
    {
        guess = 0.5f * (guess + x / guess);
        if((guess - x / guess) < epsilon)
        {
            break;
        }
    }

    return guess;
}

float my_asin(x)
float x;
{
    float neg;
    float result;
    float temp;

    neg = 0;

    if(x > 1.0f)
    {
        x = 1.0f;
    }

    if(x < -1.0f)
    {
        x = -1.0f;
    }

    if(x < 0)
    {
        neg = 1;
        x = -x;
    }

    if(x > 0.7071f)
    {
        temp = 1.0f - x;
        result = 1.57079633f - 0.5f * temp * (1.0f + temp * (0.166666f + temp * 0.075f));
    }
    else
    {
        temp = x * x;
        result = x * (1.0f + temp * (0.166666f + temp * (0.075f + temp * 0.04464f)));
    }

    if(neg)
    {
        result = -result;
    }

    return result;
}

float my_atan2(y, x)
float y;
float x;
{
    float abs_x;
    float abs_y;
    float angle;
    float r;

    if(x == 0.0f && y == 0.0f)
    {
        return 0.0f;
    }

    if(x >= 0)
    {
        abs_x = x;
    }
    else
    {
        abs_x = -x;
    }

    if(y >= 0)
    {
        abs_y = y;
    }
    else
    {
        abs_y = -y;
    }

    if(abs_y > abs_x)
    {
        r = abs_x / abs_y;
        angle = 1.57079633f - r * (0.1963f + r * -0.9817f);
    }
    else
    {
        r = abs_y / abs_x;
        angle = r * (0.1963f + r * -0.9817f);
    }

    if(x < 0)
    {
        angle = 3.14159265f - angle;
    }

    if(y < 0)
    {
        angle = -angle;
    }

    return angle;
}

float Fast_InvSqrt(x)
float x;
{
    if(x <= 0.0f)
    {
        return 0.0f;
    }
    return 1.0f / my_sqrt(x);
}

void Mahony_Init(ahrs)
struct MahonyAHRS_t *ahrs;
{
    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;
    ahrs->integralFBx = 0.0f;
    ahrs->integralFBy = 0.0f;
    ahrs->integralFBz = 0.0f;
    ahrs->inited = 1;
}

void Mahony_Update(ahrs, gx, gy, gz, ax, ay, az, dt)
struct MahonyAHRS_t *ahrs;
float gx;
float gy;
float gz;
float ax;
float ay;
float az;
float dt;
{
    float q0;
    float q1;
    float q2;
    float q3;
    float norm;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;
    float ez;
    float q0_last;
    float q1_last;
    float q2_last;
    float q3_last;

    q0 = ahrs->q0;
    q1 = ahrs->q1;
    q2 = ahrs->q2;
    q3 = ahrs->q3;

    norm = Fast_InvSqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    ahrs->integralFBx = ahrs->integralFBx + MAHONY_KI * ex * dt;
    ahrs->integralFBy = ahrs->integralFBy + MAHONY_KI * ey * dt;
    ahrs->integralFBz = ahrs->integralFBz + MAHONY_KI * ez * dt;

    gx = gx + MAHONY_KP * ex + ahrs->integralFBx;
    gy = gy + MAHONY_KP * ey + ahrs->integralFBy;
    gz = gz + MAHONY_KP * ez + ahrs->integralFBz;

    q0_last = q0 - 0.5f * (q1 * gx + q2 * gy + q3 * gz) * dt;
    q1_last = q1 + 0.5f * (q0 * gx + q2 * gz - q3 * gy) * dt;
    q2_last = q2 + 0.5f * (q0 * gy - q1 * gz + q3 * gx) * dt;
    q3_last = q3 + 0.5f * (q0 * gz + q1 * gy - q2 * gx) * dt;

    norm = Fast_InvSqrt(q0_last * q0_last + q1_last * q1_last + q2_last * q2_last + q3_last * q3_last);
    ahrs->q0 = q0_last * norm;
    ahrs->q1 = q1_last * norm;
    ahrs->q2 = q2_last * norm;
    ahrs->q3 = q3_last * norm;
}

void Quaternion_ToEuler(quat, euler)
struct Quaternion_t *quat;
struct EulerAngle_t *euler;
{
    float q0;
    float q1;
    float q2;
    float q3;
    float pitch_arg;

    q0 = quat->q0;
    q1 = quat->q1;
    q2 = quat->q2;
    q3 = quat->q3;

    euler->roll = my_atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));

    pitch_arg = 2.0f * (q0 * q2 - q3 * q1);
    if(pitch_arg > 1.0f)
    {
        pitch_arg = 1.0f;
    }
    if(pitch_arg < -1.0f)
    {
        pitch_arg = -1.0f;
    }
    euler->pitch = my_asin(pitch_arg);

    euler->yaw = my_atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

    euler->roll = euler->roll * RAD_TO_DEG;
    euler->pitch = euler->pitch * RAD_TO_DEG;
    euler->yaw = euler->yaw * RAD_TO_DEG;
}

void Mahony_GetEuler(ahrs, euler)
struct MahonyAHRS_t *ahrs;
struct EulerAngle_t *euler;
{
    struct Quaternion_t quat;
    quat.q0 = ahrs->q0;
    quat.q1 = ahrs->q1;
    quat.q2 = ahrs->q2;
    quat.q3 = ahrs->q3;
    Quaternion_ToEuler(&quat, euler);
}
