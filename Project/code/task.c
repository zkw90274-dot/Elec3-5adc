#include "task.h"

//-------------------------------------------------------------------------------------------------------------------
// 全局变量
//-------------------------------------------------------------------------------------------------------------------
static uint8 current_control_mode = CONTROL_MODE_PID_ONLY;  // 当前控制模式

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
// 函数简介     纯PID速度环控制(无循迹)
// 参数说明     void
// 返回参数     float           电机差速修正值(始终为0)
// 使用示例     float correction = pid_only_control();
// 备注信息     仅使用电机速度环PID，无循迹功能
//              用于测试电机速度环是否正常工作
//-------------------------------------------------------------------------------------------------------------------
float pid_only_control(void)
{
    // 纯PID模式，无循迹修正
    return 0.0f;
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
// 函数简介     获取当前控制模式
// 参数说明     void
// 返回参数     uint8           当前控制模式
// 使用示例     uint8 mode = get_control_mode();
// 备注信息     返回当前控制模式编号
//-------------------------------------------------------------------------------------------------------------------
uint8 get_control_mode(void)
{
    #if(CONTROL_SELECT_METHOD == CONTROL_SELECT_METHOD_STATIC)
        return CONTROL_MODE_CURRENT;
    #else
        return current_control_mode;
    #endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取控制模式名称
// 参数说明     mode            控制模式编号
// 返回参数     char*           模式名称字符串
// 使用示例     printf("当前模式: %s\r\n", get_mode_name(get_control_mode()));
// 备注信息     返回控制模式的中文名称
//-------------------------------------------------------------------------------------------------------------------
char* get_mode_name(uint8 mode)
{
    switch(mode)
    {
        case CONTROL_MODE_PID_ONLY:
            return "纯PID速度环";
        case CONTROL_MODE_SDSD:
            return "SDSD循迹";
        case CONTROL_MODE_PD_DIRECTION:
            return "PD方向环+角速度环";
        default:
            return "未知模式";
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制任务(10ms周期调用)
// 参数说明     void
// 返回参数     void
// 使用示例     motor_control_task();
// 备注信息     在10ms定时中断中调用，完成编码器采集和电机控制
//              功能: 编码器采集 -> 控制算法选择 -> 电机输出
//              控制模式由 task.h 中的 CONTROL_MODE_CURRENT 决定
//-------------------------------------------------------------------------------------------------------------------
void motor_control_task(void)
{
    float correction;
    uint8 mode;

    // 使用滤波后的编码器获取函数 (一阶IIR低通滤波)
    Encoder_Get_Filtered();

    // 获取当前控制模式
    mode = get_control_mode();

    // 根据控制模式选择不同的控制算法
    switch(mode)
    {
        case CONTROL_MODE_PID_ONLY:
            // 模式0: 纯PID速度环控制(无循迹)
            correction = pid_only_control();
            break;

        case CONTROL_MODE_SDSD:
            // 模式1: SDSD控制
            correction = sdsd_control_with_sensor_check();
            break;

        case CONTROL_MODE_PD_DIRECTION:
            // 模式2: PD方向环+角速度环控制
            correction = pd_control_with_sensor_check();
            break;

        default:
            // 默认: 纯PID模式
            correction = pid_only_control();
            break;
    }

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
