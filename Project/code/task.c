#include "task.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查ADC传感器有效性
// 参数说明     void
// 返回参数     uint8           1-传感器有效 0-传感器无效
// 使用示例     uint8 valid = sensor_check_valid();
// 备注信息     检查传感器是否全黑或全白，判断传感器数据是否可用
//-------------------------------------------------------------------------------------------------------------------
uint8 sensor_check_valid(void)
{
    uint8 sensor_valid;

    sensor_valid = 1;

    // 判断1: 检查传感器是否全黑(可能脱离赛道)
    if(adc_normalized_list[2] > 48 &&
       adc_normalized_list[1] > 48 &&
       adc_normalized_list[3] > 48)
    {
        sensor_valid = 0;  // 赛道丢失
    }

    // 判断2: 检查传感器是否全白(可能起飞或传感器故障)
    if(adc_normalized_list[2] < 2 &&
       adc_normalized_list[1] < 2 &&
       adc_normalized_list[3] < 2)
    {
        sensor_valid = 0;  // 传感器异常
    }

    return sensor_valid;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算位置误差
// 参数说明     void
// 返回参数     float           位置误差(中心为0)
// 使用示例     float err = position_error_calc();
// 备注信息     计算中间传感器相对中心的偏差
//              adc_normalized_list[2]: 0=赛道最左, 25=赛道中心, 50=赛道最右
//              返回值: 负数=偏左, 正数=偏右, 0=居中
//-------------------------------------------------------------------------------------------------------------------
float position_error_calc(void)
{
    float error;

    error = (float)adc_normalized_list[2] - 25.0f;

    return error;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     SDSD控制(带传感器判断)
// 参数说明     void
// 返回参数     float           电机差速修正值
// 使用示例     float correction = sdsd_control_with_sensor_check();
// 备注信息     使用SDSD算法计算偏差，通过PID输出电机差速修正值
//              控制流程: 传感器检查 -> SDSD计算 -> PID控制 -> 异常处理
//-------------------------------------------------------------------------------------------------------------------
float sdsd_control_with_sensor_check(void)
{
    uint8 sensor_valid;
    float input_SDSD;
    float output_SDSD;

    // 检查传感器有效性
    sensor_valid = sensor_check_valid();

    if(sensor_valid)
    {
        // 传感器正常，使用SDSD计算
        input_SDSD = SDSD_calculate(&SDSD);
        output_SDSD = pid_calc_position(&pid_SDSD, input_SDSD);
    }
    else
    {
        // 传感器异常，保持上一次的PID输出
        output_SDSD = pid_SDSD.out;
    }

    return output_SDSD;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PD方向环+角速度环控制(带传感器判断)
// 参数说明     void
// 返回参数     float           电机差速修正值
// 使用示例     float correction = pd_control_with_sensor_check();
// 备注信息     自动检查传感器有效性，传感器异常时使用IMU角速度反馈保持直行
//              控制流程: 传感器检查 -> 位置误差计算 -> PD控制 -> 异常处理
//-------------------------------------------------------------------------------------------------------------------
float pd_control_with_sensor_check(void)
{
    uint8 sensor_valid;
    float position_error;
    float correction;

    // 检查传感器有效性
    sensor_valid = sensor_check_valid();

    // 计算位置误差
    position_error = position_error_calc();

    // PD方向环+角速度环组合控制
    if(sensor_valid)
    {
        // 传感器正常，使用PD控制
        correction = pd_direction_gyro_loop(position_error);
    }
    else
    {
        // 传感器异常，使用IMU角速度反馈保持直行
        correction = -imu.gyro_z * 0.5f;  // 简单的角速度负反馈
    }

    return correction;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制任务(10ms周期调用)
// 参数说明     void
// 返回参数     void
// 使用示例     motor_control_task();
// 备注信息     在10ms定时中断中调用，完成编码器采集和电机控制
//              功能: 编码器采集 -> 控制算法选择 -> 电机输出
//              控制模式由 CONTROL_MODE_CURRENT 宏定义决定
//-------------------------------------------------------------------------------------------------------------------
void motor_control_task(void)
{
    float correction;

    // 使用滤波后的编码器获取函数 (一阶IIR低通滤波)
    Encoder_Get_Filtered();

    // 根据控制模式选择不同的控制算法
#if(CONTROL_MODE_CURRENT == CONTROL_MODE_SDSD)
    // 模式0: SDSD控制
    correction = sdsd_control_with_sensor_check();

#elif(CONTROL_MODE_CURRENT == CONTROL_MODE_PD_DIRECTION)
    // 模式1: PD方向环+角速度环控制
    correction = pd_control_with_sensor_check();

#else
    #error "未定义的控制模式，请在 task.h 中设置 CONTROL_MODE_CURRENT"
#endif

    // 电机PID控制 (修正值叠加到编码器输入)
    motor_pid_control(encoder_data_dir_L + correction, encoder_data_dir_R - correction);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU更新任务(50ms周期调用)
// 参数说明     void
// 返回参数     void
// 使用示例     imu_update_task();
// 备注信息     在50ms定时中断中调用，更新IMU数据
//              功能: 获取IMU原始数据 -> 四元数解算 -> 欧拉角更新
//-------------------------------------------------------------------------------------------------------------------
void imu_update_task(void)
{
    imu_update();
}
