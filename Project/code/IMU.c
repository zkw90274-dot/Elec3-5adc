//头文件包含
#include "IMU.h"

struct IMUData imu;
struct MahonyAHRS_t ahrs;

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
// 备注信息     初始化IMU660RB传感器和Mahony姿态解算器
//-------------------------------------------------------------------------------------------------------------------
uint8 imu_init(void)
{
    uint8 result;

    // 初始化IMU660RB传感器
    result = imu660rb_init();

    if(result == 0)
    {
        // 等待传感器稳定
        system_delay_ms(100);

        // 初始化Mahony姿态解算器
        Mahony_Init(&ahrs);

        // 获取初始数据，用于预对准
        imu_get_data();

        // 使用加速度计初始值对准姿态（可选）
        // 如果传感器水平放置，可以跳过这一步
        printf("IMU init success!\r\n");
    }
    else
    {
        printf("IMU init failed!\r\n");
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
// 函数简介     IMU数据更新
// 参数说明     void
// 返回参数     void
// 使用示例     imu_update();
// 备注信息     获取原始数据并使用Mahony算法进行四元数解算
//              需要周期调用，建议10ms(100Hz)或根据IMU_SAMPLE_RATE调整
//-------------------------------------------------------------------------------------------------------------------
void imu_update(void)
{
    float ax, ay, az;
    float gx, gy, gz;
    float norm;

    // 1. 获取IMU原始数据
    imu_get_data();

    // 2. 归一化加速度计数据（Mahony算法要求输入归一化后的加速度）
    norm = Fast_InvSqrt(imu.acc_x * imu.acc_x + imu.acc_y * imu.acc_y + imu.acc_z * imu.acc_z);
    ax = imu.acc_x * norm;
    ay = imu.acc_y * norm;
    az = imu.acc_z * norm;

    // 3. 陀螺仪数据转换为弧度/秒
    gx = imu.gyro_x * DEG_TO_RAD;
    gy = imu.gyro_y * DEG_TO_RAD;
    gz = imu.gyro_z * DEG_TO_RAD;

    // 4. 使用Mahony算法更新姿态
    Mahony_Update(&ahrs, gx, gy, gz, ax, ay, az, DT);

    // 5. 获取四元数结果
    imu.q0 = ahrs.q0;
    imu.q1 = ahrs.q1;
    imu.q2 = ahrs.q2;
    imu.q3 = ahrs.q3;

    // 6. 转换为欧拉角
    Mahony_GetEuler(&ahrs, (struct EulerAngle_t *)&imu.roll);

    // 7. 归一化角度到-180~180度范围
    imu.roll = angle_normalize(imu.roll);
    imu.pitch = angle_normalize(imu.pitch);
    imu.yaw = angle_normalize(imu.yaw);
}
