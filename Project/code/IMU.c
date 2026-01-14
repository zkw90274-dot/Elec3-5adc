//头文件包含
#include "IMU.h"


// 全局变量定义
IMUData imu;

//互补滤波系数
#define COMP_FILTER_Kp           0.5f             // 互补滤波比例增益
#define COMP_FILTER_Ki           0.001f           // 互补滤波积分增益

// 互补滤波积分误差累计量
static float ex_int = 0, ey_int = 0, ez_int = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     反正弦函数 (实现asinf)
// 参数说明     x               输入值 [-1, 1]
// 返回参数     float           反正弦值 (弧度)
// 备注信息     使用多项式近似实现
//-------------------------------------------------------------------------------------------------------------------
static float my_asin(float x)
{
    float neg, result, temp;

    neg = 0;
    if(x > 1.0f)
        x = 1.0f;
    if(x < -1.0f)
        x = -1.0f;

    if(x < 0)
    {
        neg = 1;
        x = -x;
    }

    // 使用多项式近似，避免递归
    if(x > 0.7071f)
    {
        // 对于接近1的值，使用变换后的多项式
        temp = 1.0f - x;
        result = 1.57079633f - 0.5f * temp * (1.0f + temp * (0.166666f + temp * 0.075f));
    }
    else
    {
        temp = x * x;
        result = x * (1.0f + temp * (0.166666f + temp * (0.075f + temp * 0.04464f)));
    }

    if(neg)
        result = -result;

    return result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     反正切函数 (实现atan2f)
// 参数说明     y               y坐标
// 参数说明     x               x坐标
// 返回参数     float           反正切值 (弧度)
// 备注信息     实现atan2(y,x)功能
//-------------------------------------------------------------------------------------------------------------------
static float my_atan2(float y, float x)
{
    float abs_x, abs_y, angle, r;

    if(x == 0.0f && y == 0.0f)
        return 0.0f;

    if(x >= 0)
        abs_x = x;
    else
        abs_x = -x;

    if(y >= 0)
        abs_y = y;
    else
        abs_y = -y;

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
        angle = 3.14159265f - angle;

    if(y < 0)
        angle = -angle;

    return angle;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     快速平方根倒数算法
// 参数说明     x               输入值
// 返回参数     float           1/sqrt(x)
// 使用示例     float y = fast_inv_sqrt(x);
// 备注信息     使用快速算法计算平方根倒数
//-------------------------------------------------------------------------------------------------------------------
float fast_inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角度归一化到-180~180度范围
// 参数说明     angle           角度值
// 返回参数     float           归一化后的角度(-180~180)
// 使用示例     float angle = angle_normalize(angle);
// 备注信息     将角度归一化到-180~180度范围
//-------------------------------------------------------------------------------------------------------------------
float angle_normalize(float angle)
{
    while(angle > 180.0f)
        angle -= 360.0f;
    while(angle < -180.0f)
        angle += 360.0f;
    return angle;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU初始化
// 参数说明     void
// 返回参数     uint8           0-初始化成功 1-初始化失败
// 使用示例     uint8 result = imu_init();
// 备注信息     内部调用imu660rb_init()
//-------------------------------------------------------------------------------------------------------------------
uint8 imu_init(void)
{
    uint8 result;

    // 初始化imu660rb
    result = imu660rb_init();

    if(result == 0)
    {
        // 获取初始数据用于四元数初始化
        system_delay_ms(50);
        imu_get_data();

        // 使用加速度计初始值初始化四元数
        imu_quaternion_init();
    }

    return result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取IMU原始数据
// 参数说明     void
// 返回参数     void
// 使用示例     imu_get_data();
// 备注信息     从imu660rb获取加速度计和陀螺仪数据并转换为物理单位
//-------------------------------------------------------------------------------------------------------------------
void imu_get_data(void)
{
    // 获取原始数据
    imu660rb_get_acc();
    imu660rb_get_gyro();

    // 转换为物理单位
    imu.acc_x = imu660rb_acc_transition(imu660rb_acc_x);
    imu.acc_y = imu660rb_acc_transition(imu660rb_acc_y);
    imu.acc_z = imu660rb_acc_transition(imu660rb_acc_z);

    imu.gyro_x = imu660rb_gyro_transition(imu660rb_gyro_x);
    imu.gyro_y = imu660rb_gyro_transition(imu660rb_gyro_y);
    imu.gyro_z = imu660rb_gyro_transition(imu660rb_gyro_z);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数初始化
// 参数说明     void
// 返回参数     void
// 使用示例     imu_quaternion_init();
// 备注信息     使用加速度计初始值初始化四元数
//-------------------------------------------------------------------------------------------------------------------
void imu_quaternion_init(void)
{
    float norm;
    float halfvx, halfvy, halfvz;

    // 归一化加速度计向量
    norm = fast_inv_sqrt(imu.acc_x * imu.acc_x + imu.acc_y * imu.acc_y + imu.acc_z * imu.acc_z);
    imu.acc_x *= norm;
    imu.acc_y *= norm;
    imu.acc_z *= norm;

    // 初始化四元数，假设设备水平静止
    // 使用加速度计数据初始化pitch和roll
    halfvx = imu.acc_x;
    halfvy = imu.acc_y;
    halfvz = imu.acc_z;

    // 计算初始四元数
    if(halfvz >= 0.0f)
    {
        imu.quat.q0 = fast_inv_sqrt(2.0f * (1.0f + halfvz));
        imu.quat.q3 = 0.5f * imu.quat.q0;
        imu.quat.q1 = -halfvy * imu.quat.q3;
        imu.quat.q2 = halfvx * imu.quat.q3;
    }
    else
    {
        imu.quat.q3 = fast_inv_sqrt(2.0f * (1.0f - halfvz));
        imu.quat.q0 = 0.5f * imu.quat.q3;
        imu.quat.q1 = halfvy * imu.quat.q0;
        imu.quat.q2 = -halfvx * imu.quat.q0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数更新(互补滤波算法)
// 参数说明     void
// 返回参数     void
// 使用示例     imu_quaternion_update();
// 备注信息     使用互补滤波算法融合加速度计和陀螺仪数据更新四元数
//-------------------------------------------------------------------------------------------------------------------
void imu_quaternion_update(void)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float q0_last, q1_last, q2_last, q3_last;
    float gx_rad, gy_rad, gz_rad;

    // 归一化加速度计测量值
    norm = fast_inv_sqrt(imu.acc_x * imu.acc_x + imu.acc_y * imu.acc_y + imu.acc_z * imu.acc_z);
    imu.acc_x *= norm;
    imu.acc_y *= norm;
    imu.acc_z *= norm;

    // 估计重力方向
    vx = 2.0f * (imu.quat.q1 * imu.quat.q3 - imu.quat.q0 * imu.quat.q2);
    vy = 2.0f * (imu.quat.q0 * imu.quat.q1 + imu.quat.q2 * imu.quat.q3);
    vz = imu.quat.q0 * imu.quat.q0 - imu.quat.q1 * imu.quat.q1 - imu.quat.q2 * imu.quat.q2 + imu.quat.q3 * imu.quat.q3;

    // 计算误差
    ex = (imu.acc_y * vz - imu.acc_z * vy);
    ey = (imu.acc_z * vx - imu.acc_x * vz);
    ez = (imu.acc_x * vy - imu.acc_y * vx);

    // 积累误差
    ex_int += ex * COMP_FILTER_Ki;
    ey_int += ey * COMP_FILTER_Ki;
    ez_int += ez * COMP_FILTER_Ki;

    // 调整后的陀螺仪数据
    imu.gyro_x += COMP_FILTER_Kp * ex + ex_int;
    imu.gyro_y += COMP_FILTER_Kp * ey + ey_int;
    imu.gyro_z += COMP_FILTER_Kp * ez + ez_int;

    // 将角速度转换为弧度/s
    gx_rad = imu.gyro_x * 0.0174533f;  // deg/s to rad/s
    gy_rad = imu.gyro_y * 0.0174533f;
    gz_rad = imu.gyro_z * 0.0174533f;

    // 保存当前四元数
    q0_last = imu.quat.q0;
    q1_last = imu.quat.q1;
    q2_last = imu.quat.q2;
    q3_last = imu.quat.q3;

    // 四元数微分方程
    imu.quat.q0 += (-q1_last * gx_rad - q2_last * gy_rad - q3_last * gz_rad) * DT * 0.5f;
    imu.quat.q1 += ( q0_last * gx_rad + q2_last * gz_rad - q3_last * gy_rad) * DT * 0.5f;
    imu.quat.q2 += ( q0_last * gy_rad - q1_last * gz_rad + q3_last * gx_rad) * DT * 0.5f;
    imu.quat.q3 += ( q0_last * gz_rad + q1_last * gy_rad - q2_last * gx_rad) * DT * 0.5f;

    // 归一化四元数
    norm = fast_inv_sqrt(imu.quat.q0 * imu.quat.q0 + imu.quat.q1 * imu.quat.q1 +
                         imu.quat.q2 * imu.quat.q2 + imu.quat.q3 * imu.quat.q3);
    imu.quat.q0 *= norm;
    imu.quat.q1 *= norm;
    imu.quat.q2 *= norm;
    imu.quat.q3 *= norm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数转欧拉角
// 参数说明     void
// 返回参数     void
// 使用示例     imu_quaternion_to_euler();
// 备注信息     将四元数转换为欧拉角(roll,pitch,yaw)
//-------------------------------------------------------------------------------------------------------------------
void imu_quaternion_to_euler(void)
{
    float q0, q1, q2, q3;

    q0 = imu.quat.q0;
    q1 = imu.quat.q1;
    q2 = imu.quat.q2;
    q3 = imu.quat.q3;

    // 计算roll (横滚角)
    imu.euler.roll = my_atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;

    // 计算pitch (俯仰角)
    imu.euler.pitch = my_asin(2.0f * (q0 * q2 - q1 * q3)) * 57.29578f;

    // 计算yaw (偏航角)
    imu.euler.yaw = my_atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f;

    // 归一化角度到-180~180度范围
    imu.euler.roll = angle_normalize(imu.euler.roll);
    imu.euler.pitch = angle_normalize(imu.euler.pitch);
    imu.euler.yaw = angle_normalize(imu.euler.yaw);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU数据更新
// 参数说明     void
// 返回参数     void
// 使用示例     imu_update();
// 备注信息     获取原始数据并进行四元数解算，需要周期调用(建议10ms)
//-------------------------------------------------------------------------------------------------------------------
void imu_update(void)
{
    // 获取IMU原始数据
    imu_get_data();

    // 更新四元数
    imu_quaternion_update();

    // 转换为欧拉角
    imu_quaternion_to_euler();
}
