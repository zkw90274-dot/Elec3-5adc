/*********************************************************************************************************************
 * @file        quaternion.c
 * @brief       四元数姿态解算算法实现 (Mahony算法)
 * @platform    STC32G
 * @note        用于IMU姿态解算，将陀螺仪和加速度计数据转换为欧拉角
********************************************************************************************************************/

#include "quaternion.h"

/*==================================================================================================================*/
/* =============== 数学基础函数 =============== */
/*==================================================================================================================*/

/**
 * @brief       快速平方根计算 (牛顿迭代法)
 * @param       x   输入值 (必须 >= 0)
 * @return      平方根值
 * @note        使用牛顿迭代法，迭代20次或达到精度要求
 */
float my_sqrt(float x)
{
    float guess = x;
    float epsilon = 0.00001f;
    uint16 i;

    // 牛顿迭代法: guess = 0.5 * (guess + x / guess)
    for(i = 0; i < 20; i++)
    {
        guess = 0.5f * (guess + x / guess);
        if((guess - x / guess) < epsilon)
        {
            break;  // 达到精度要求，提前退出
        }
    }

    return guess;
}

/**
 * @brief       反正弦函数计算 (泰勒级数近似)
 * @param       x   输入值 [-1, 1]
 * @return      弧度值 [-π/2, π/2]
 * @note        使用分段多项式近似，提高精度
 */
float my_asin(float x)
{
    float neg;
    float result;
    float temp;

    neg = 0;

    // 输入限幅，防止越界
    if(x > 1.0f)
    {
        x = 1.0f;
    }

    if(x < -1.0f)
    {
        x = -1.0f;
    }

    // 处理负数情况 (asin(-x) = -asin(x))
    if(x < 0)
    {
        neg = 1;
        x = -x;
    }

    // 分段多项式近似计算
    if(x > 0.7071f)  // > 45度
    {
        // π/2 - 0.5 * (1-x) * (1 + (1-x) * (1/6 + (1-x) * 0.075))
        temp = 1.0f - x;
        result = 1.57079633f - 0.5f * temp * (1.0f + temp * (0.166666f + temp * 0.075f));
    }
    else  // <= 45度
    {
        // x * (1 + x² * (1/6 + x² * (0.075 + x² * 0.04464)))
        temp = x * x;
        result = x * (1.0f + temp * (0.166666f + temp * (0.075f + temp * 0.04464f)));
    }

    // 恢复符号
    if(neg)
    {
        result = -result;
    }

    return result;
}

/**
 * @brief       反正切函数 (双参数版本)
 * @param       y   Y坐标
 * @param       x   X坐标
 * @return      弧度值 [-π, π]
 * @note        根据象限返回正确的角度
 */
float my_atan2(float y, float x)
{
    float abs_x;
    float abs_y;
    float angle;
    float r;

    // 原点处理
    if(x == 0.0f && y == 0.0f)
    {
        return 0.0f;
    }

    // 取绝对值
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

    // 多项式近似 atan(r)
    if(abs_y > abs_x)  // |y| > |x|
    {
        r = abs_x / abs_y;
        angle = 1.57079633f - r * (0.1963f + r * -0.9817f);  // π/2 - ...
    }
    else  // |x| >= |y|
    {
        r = abs_y / abs_x;
        angle = r * (0.1963f + r * -0.9817f);
    }

    // 象限修正
    if(x < 0)  // 第二、三象限
    {
        angle = 3.14159265f - angle;  // π - angle
    }

    if(y < 0)  // 第三、四象限
    {
        angle = -angle;
    }

    return angle;
}

/**
 * @brief       快速平方根倒数
 * @param       x   输入值 (> 0)
 * @return      1/sqrt(x)
 * @note        用于向量归一化
 */
float Fast_InvSqrt(float x)
{
    if(x <= 0.0f)
    {
        return 0.0f;
    }
    return 1.0f / my_sqrt(x);
}

/*==================================================================================================================*/
/* =============== Mahony 姿态解算算法 =============== */
/*==================================================================================================================*/

/**
 * @brief       Mahony AHRS 算法初始化
 * @param       ahrs    Mahony算法结构体指针
 * @note        初始化四元数为单位姿态 (无旋转)
 */
void Mahony_Init(struct MahonyAHRS_t *ahrs)
{
    // 初始化为单位四元数 (1, 0, 0, 0) 表示无旋转
    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;

    // 积分误差清零
    ahrs->integralFBx = 0.0f;
    ahrs->integralFBy = 0.0f;
    ahrs->integralFBz = 0.0f;

    ahrs->inited = 1;
}

/**
 * @brief       Mahony AHRS 算法更新 (融合陀螺仪和加速度计数据)
 * @param       ahrs    Mahony算法结构体指针
 * @param       gx      陀螺仪X轴角速度 (rad/s)
 * @param       gy      陀螺仪Y轴角速度 (rad/s)
 * @param       gz      陀螺仪Z轴角速度 (rad/s)
 * @param       ax      加速度计X轴加速度
 * @param       ay      加速度计Y轴加速度
 * @param       az      加速度计Z轴加速度
 * @param       dt      采样时间间隔 (s)
 * @note        互补滤波融合: 陀螺仪用于高频响应，加速度计消除漂移
 */
void Mahony_Update(struct MahonyAHRS_t *ahrs, float gx, float gy, float gz,
                   float ax, float ay, float az, float dt)
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

    // 读取当前四元数
    q0 = ahrs->q0;
    q1 = ahrs->q1;
    q2 = ahrs->q2;
    q3 = ahrs->q3;

    // ========== 步骤1: 加速度计归一化 ==========
    norm = Fast_InvSqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    // ========== 步骤2: 计算重力加速度参考向量 ==========
    // 根据当前四元数计算机体系下的重力向量
    // 重力在世界坐标系下为 (0, 0, 1)，旋转到机体系为:
    vx = 2.0f * (q1 * q3 - q0 * q2);  // wx = 2(q1q3 - q0q2)
    vy = 2.0f * (q0 * q1 + q2 * q3);  // wy = 2(q0q1 + q2q3)
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;  // wz = q0² - q1² - q2² + q3²

    // ========== 步骤3: 计算方向误差 (加速度计 vs 重力) ==========
    // 叉积: 测量加速度 × 重力参考方向
    ex = ay * vz - az * vy;  // ay*wz - az*wy
    ey = az * vx - ax * vz;  // az*wx - az*wz
    ez = ax * vy - ay * vx;  // ax*wy - ay*wx

    // ========== 步骤4: 积分误差累积 (PI控制器中的I) ==========
    // 用于消除陀螺仪漂移
    ahrs->integralFBx = ahrs->integralFBx + MAHONY_KI * ex * dt;
    ahrs->integralFBy = ahrs->integralFBy + MAHONY_KI * ey * dt;
    ahrs->integralFBz = ahrs->integralFBz + MAHONY_KI * ez * dt;

    // ========== 步骤5: 修正角速度 (PI控制器: P + I) ==========
    // 比例项 + 积分项
    gx = gx + MAHONY_KP * ex + ahrs->integralFBx;
    gy = gy + MAHONY_KP * ey + ahrs->integralFBy;
    gz = gz + MAHONY_KP * ez + ahrs->integralFBz;

    // ========== 步骤6: 四元数微分方程更新 ==========
    // q' = 0.5 * q * ω (四元数导数)
    q0_last = q0 - 0.5f * (q1 * gx + q2 * gy + q3 * gz) * dt;
    q1_last = q1 + 0.5f * (q0 * gx + q2 * gz - q3 * gy) * dt;
    q2_last = q2 + 0.5f * (q0 * gy - q1 * gz + q3 * gx) * dt;
    q3_last = q3 + 0.5f * (q0 * gz + q1 * gy - q2 * gx) * dt;

    // ========== 步骤7: 四元数归一化 ==========
    norm = Fast_InvSqrt(q0_last * q0_last + q1_last * q1_last + q2_last * q2_last + q3_last * q3_last);
    ahrs->q0 = q0_last * norm;
    ahrs->q1 = q1_last * norm;
    ahrs->q2 = q2_last * norm;
    ahrs->q3 = q3_last * norm;
}

/*==================================================================================================================*/
/* =============== 四元数与欧拉角转换 =============== */
/*==================================================================================================================*/

/**
 * @brief       四元数转欧拉角 (Roll-Pitch-Yaw)
 * @param       quat    四元数结构体指针
 * @param   euler   欧拉角结构体指针
 * @note        输出单位为度 (°)
 *
 * 欧拉角定义 (ZYX顺序, 即航空顺序):
 * - Roll (横滚): 绕X轴旋转 [-180°, 180°]
 * - Pitch (俯仰): 绕Y轴旋转 [-90°, 90°]
 * - Yaw (偏航): 绕Z轴旋转 [-180°, 180°]
 */
void Quaternion_ToEuler(struct Quaternion_t *quat, struct EulerAngle_t *euler)
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

    // ========== Roll (横滚角) ==========
    // roll = atan2(2(q0q1 + q2q3), 1 - 2(q1² + q2²))
    euler->roll = my_atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));

    // ========== Pitch (俯仰角) ==========
    // pitch = asin(2(q0q2 - q3q1))
    pitch_arg = 2.0f * (q0 * q2 - q3 * q1);
    if(pitch_arg > 1.0f)  // 限幅防止越界
    {
        pitch_arg = 1.0f;
    }
    if(pitch_arg < -1.0f)
    {
        pitch_arg = -1.0f;
    }
    euler->pitch = my_asin(pitch_arg);

    // ========== Yaw (偏航角) ==========
    // yaw = atan2(2(q0q3 + q1q2), 1 - 2(q2² + q3²))
    euler->yaw = my_atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

    // 弧度转角度
    euler->roll = euler->roll * RAD_TO_DEG;
    euler->pitch = euler->pitch * RAD_TO_DEG;
    euler->yaw = euler->yaw * RAD_TO_DEG;
}

/**
 * @brief       从 Mahony AHRS 获取欧拉角
 * @param       ahrs    Mahony算法结构体指针
 * @param   euler   欧拉角结构体指针
 */
void Mahony_GetEuler(struct MahonyAHRS_t *ahrs, struct EulerAngle_t *euler)
{
    struct Quaternion_t quat;

    // 从 Mahony 结构体提取四元数
    quat.q0 = ahrs->q0;
    quat.q1 = ahrs->q1;
    quat.q2 = ahrs->q2;
    quat.q3 = ahrs->q3;

    // 转换为欧拉角
    Quaternion_ToEuler(&quat, euler);
}
