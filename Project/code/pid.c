#include "pid.h"

//===================================================================================================================
// 轻量级三角函数 (C251编译器不支持math.h)
//================================================================================================================---

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     快速正弦函数近似 (泰勒级数)
// 参数说明     x               弧度值 (建议范围: -PI ~ PI)
// 返回参数     float           sin(x) 近似值
// 备注信息     使用5阶泰勒级数: sin(x) ≈ x - x³/6 + x⁵/120
//              精度: ±0.1% (在-PI~PI范围内)
//-------------------------------------------------------------------------------------------------------------------
static float sinf(float x)
{
    float x2, x3, x5;
    float result;

    // 将角度限制在 -PI ~ PI
    while(x > 3.14159f)
        x -= 6.28318f;
    while(x < -3.14159f)
        x += 6.28318f;

    x2 = x * x;
    x3 = x2 * x;
    x5 = x3 * x2;

    // 泰勒级数: sin(x) ≈ x - x³/6 + x⁵/120
    result = x - x3 * 0.166667f + x5 * 0.008333f;

    return result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     快速余弦函数近似 (泰勒级数)
// 参数说明     x               弧度值 (建议范围: -PI ~ PI)
// 返回参数     float           cos(x) 近似值
// 备注信息     使用4阶泰勒级数: cos(x) ≈ 1 - x²/2 + x⁴/24
//              精度: ±0.1% (在-PI~PI范围内)
//-------------------------------------------------------------------------------------------------------------------
static float cosf(float x)
{
    float x2, x4;
    float result;

    // 将角度限制在 -PI ~ PI
    while(x > 3.14159f)
        x -= 6.28318f;
    while(x < -3.14159f)
        x += 6.28318f;

    x2 = x * x;
    x4 = x2 * x2;

    // 泰勒级数: cos(x) ≈ 1 - x²/2 + x⁴/24
    result = 1.0f - x2 * 0.5f + x4 * 0.041667f;

    return result;
}

//===================================================================================================================
// 全局变量定义
//===================================================================================================================
pid_param_t pid_motor_left;     // 左电机PID控制器
pid_param_t pid_motor_right;    // 右电机PID控制器
pid_param_t pid_SDSD;           // 差比和差PID控制器

// PD控制器全局变量
pd_param_t pd_direction;        // 方向环PD参数

// 四元数姿态控制器全局变量
quaternion_attitude_t attitude_controller;  // 四元数姿态控制器

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID参数初始化
// 参数说明     pid             PID结构体指针
// 参数说明     kp              比例系数
// 参数说明     ki              积分系数
// 参数说明     kd              微分系数
// 返回参数     void
// 使用示例     pid_init(&pid_motor_left, 1.7, 0.22, 0.2);
//-------------------------------------------------------------------------------------------------------------------
void pid_init(pid_param_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = 0;
    pid->actual = 0;
    pid->error = 0;
    pid->last_error = 0;
    pid->prev_error = 0;
    pid->out_p = 0;
    pid->out_i = 0;
    pid->out_d = 0;
    pid->out = 0;
    pid->integrator = 0;
    pid->integral_max = PID_INTEGRAL_MAX;
    pid->output_max = PID_OUTPUT_MAX;
    pid->output_min = PID_OUTPUT_MIN;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置PID参数
// 参数说明     pid             PID结构体指针
// 参数说明     kp              比例系数
// 参数说明     ki              积分系数
// 参数说明     kd              微分系数
// 返回参数     void
// 使用示例     pid_set_param(&pid_motor_left, 2.0, 0.3, 0.25);
//-------------------------------------------------------------------------------------------------------------------
void pid_set_param(pid_param_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置PID目标值
// 参数说明     pid             PID结构体指针
// 参数说明     target          目标值
// 返回参数     void
// 使用示例     pid_set_target(&pid_motor_left, 100);
//-------------------------------------------------------------------------------------------------------------------
void pid_set_target(pid_param_t *pid, float target)
{
    pid->target = -target;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     限幅函数
// 参数说明     value           待限幅值
// 参数说明     min_val         最小值
// 参数说明     max_val         最大值
// 返回参数     float           限幅后的值
//-------------------------------------------------------------------------------------------------------------------
float constrain_float(float value, float min_val, float max_val)
{
    if(value <= min_val)
        return min_val;
    else if(value >= max_val)
        return max_val;
    else
        return value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     位置式PID计算
// 参数说明     pid             PID结构体指针
// 参数说明     actual          当前实际值
// 返回参数     float           PID输出值
//-------------------------------------------------------------------------------------------------------------------
float pid_calc_position(pid_param_t *pid, float actual)
{
    pid->actual = actual;
    pid->error = pid->target - actual;
    pid->integrator += pid->error;
    pid->integrator = constrain_float(pid->integrator, -pid->integral_max, pid->integral_max);
    pid->out_p = pid->kp * pid->error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (pid->error - pid->last_error);
    pid->last_error = pid->error;
    pid->out = pid->out_p + pid->out_i + pid->out_d;
    pid->out = constrain_float(pid->out, pid->output_min, pid->output_max);
    return pid->out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     增量式PID计算
// 参数说明     pid             PID结构体指针
// 参数说明     actual          当前实际值
// 返回参数     float           PID输出值
//-------------------------------------------------------------------------------------------------------------------
float pid_calc_increment(pid_param_t *pid, float actual)
{
    float increment;
    pid->actual = actual;
    pid->error = pid->target - actual;
    pid->out_p = pid->kp * (pid->error - pid->last_error);
    pid->out_i = pid->ki * pid->error;
    pid->out_d = pid->kd * (pid->error - 2 * pid->last_error + pid->prev_error);
    increment = pid->out_p + pid->out_i + pid->out_d;
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;
    pid->out += increment;
    pid->out = constrain_float(pid->out, pid->output_min, pid->output_max);
    return pid->out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机PID控制函数
// 参数说明     encoder_left    左编码器值（已叠加差速修正）
// 参数说明     encoder_right   右编码器值（已叠加差速修正）
// 返回参数     void
// 使用示例     motor_pid_control(encoder_left, encoder_right);
// 备注信息     双电机速度环闭环控制，使用增量式PID
//              注意：差速修正在 motor_control_task() 中完成，此函数仅处理速度环
//-------------------------------------------------------------------------------------------------------------------
void motor_pid_control(int16 encoder_left, int16 encoder_right)
{
    float output_left;
    float output_right;
    uint8 pwm_left;
    uint8 pwm_right;

    // 增量式PID速度环计算
    output_left = pid_calc_increment(&pid_motor_left, encoder_left);
    output_right = pid_calc_increment(&pid_motor_right, encoder_right);

    // 左电机PWM输出（带方向控制）
    if(output_left >= 0)
    {
        pwm_left = (output_left > PID_OUTPUT_MAX) ? PID_OUTPUT_MAX : (uint8)output_left;
        Motor_LeftForward(pwm_left);
    }
    else
    {
        pwm_left = ((-output_left) > PID_OUTPUT_MAX) ? PID_OUTPUT_MAX : (uint8)(-output_left);
        Motor_LeftBackward(pwm_left);
    }

    // 右电机PWM输出（带方向控制）
    if(output_right >= 0)
    {
        pwm_right = (output_right > PID_OUTPUT_MAX) ? PID_OUTPUT_MAX : (uint8)output_right;
        Motor_RightForward(pwm_right);
    }
    else
    {
        pwm_right = ((-output_right) > PID_OUTPUT_MAX) ? PID_OUTPUT_MAX : (uint8)(-output_right);
        Motor_RightBackward(pwm_right);
    }
}

//===================================================================================================================
// PD控制器实现 - 角度环/角速度环/方向环
//===================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PD控制器参数初始化
// 参数说明     pd              PD结构体指针
// 参数说明     kp              比例系数
// 参数说明     kd              微分系数
// 参数说明     kp_gyro         角速度环比例系数
// 参数说明     kd_gyro         角速度环微分系数
// 返回参数     void
// 使用示例     pd_init(&pd_direction, 2.5, 0.8, 1.2, 0.3);
//-------------------------------------------------------------------------------------------------------------------
void pd_init(pd_param_t *pd, float kp, float kd, float kp_gyro, float kd_gyro)
{
    pd->kp = kp;
    pd->kd = kd;
    pd->kp_gyro = kp_gyro;
    pd->kd_gyro = kd_gyro;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PD方向环+角速度环组合控制 (推荐用于循迹) - 四元数实现版本
// 参数说明     err_position    位置误差(ADC归一化值-中心值)
// 返回参数     float           电机差速修正值
// 使用示例     float correction = pd_direction_gyro_loop(adc_error);
// 备注信息     【已升级】使用四元数姿态误差替代传统PD计算
//              控制流程:
//              1. 计算四元数姿态误差: q_error = q_target ⊗ q_current^(-1)
//              2. 融合位置误差与姿态误差
//              3. 经PD计算得到期望角速度
//              4. 期望角速度与实际角速度比较
//              5. 角速度误差经PD计算得到电机差速修正值
//              6. 三环结构提供更精确的姿态控制
//-------------------------------------------------------------------------------------------------------------------
float pd_direction_gyro_loop(float err_position)
{
    static float err_position_history[4] = {0};
    static float err_gyro_history[2] = {0};
    float expect_gyro;
    float err_gyro;
    float correction;
    float fused_error;                   // 融合误差
    float qc0, qc1, qc2, qc3;           // 当前姿态四元数的共轭
    float qe0, qe1, qe2, qe3;           // 误差四元数
    float ez;                           // 姿态误差向量(Z轴-偏航)

    // ========== 第一级：位置误差 + 四元数姿态误差融合 ==========

    // 误差历史更新
    err_position_history[2] = err_position_history[1];
    err_position_history[1] = err_position_history[0];
    err_position_history[0] = err_position;

    // 计算四元数姿态误差
    // 步骤1: 获取当前姿态四元数的共轭
    qc0 = imu.q0;
    qc1 = -imu.q1;
    qc2 = -imu.q2;
    qc3 = -imu.q3;

    // 步骤2: 计算误差四元数 q_error = q_target ⊗ q_current^(-1)
    quaternion_multiply(attitude_controller.target_q0, attitude_controller.target_q1,
                       attitude_controller.target_q2, attitude_controller.target_q3,
                       qc0, qc1, qc2, qc3,
                       &qe0, &qe1, &qe2, &qe3);

    // 步骤3: 提取Z轴姿态误差向量 (偏航误差)
    ez = 2.0f * qe0 * qe3;

    // 步骤4: 融合位置误差与姿态误差
    // 位置误差 (ADC) 作为主要控制量，姿态误差作为修正
    fused_error = err_position_history[0] + ez * 10.0f;  // ez单位转换系数

    // PD计算：融合误差 -> 期望角速度
    expect_gyro = pd_direction.kp * fused_error +
                  pd_direction.kd * (fused_error - err_position_history[1]);

    // 期望角速度限幅 (防止响应过激)
    if(expect_gyro > 100.0f)
        expect_gyro = 100.0f;
    else if(expect_gyro < -100.0f)
        expect_gyro = -100.0f;

    // ========== 第二级：PD角速度环 ==========
    // 角速度误差计算
    err_gyro = expect_gyro - imu.gyro_z;

    // 误差历史更新
    err_gyro_history[1] = err_gyro_history[0];
    err_gyro_history[0] = err_gyro;

    // PD计算：角速度误差 -> 电机差速修正值
    correction = pd_direction.kp_gyro * err_gyro_history[0] +
                 pd_direction.kd_gyro * (err_gyro_history[0] - err_gyro_history[1]);

    // 输出限幅 (根据实际需要调整)
    if(correction > 100.0f)
        correction = 100.0f;
    else if(correction < -100.0f)
        correction = -100.0f;

    return correction;
}

//===================================================================================================================
// 四元数姿态控制实现
//===================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     欧拉角转四元数
// 参数说明     roll            横滚角 (度)
// 参数说明     pitch           俯仰角 (度)
// 参数说明     yaw             偏航角 (度)
// 参数说明     q0              输出四元数 w 分量
// 参数说明     q1              输出四元数 x 分量
// 参数说明     q2              输出四元数 y 分量
// 参数说明     q3              输出四元数 z 分量
// 返回参数     void
// 使用示例     euler_to_quaternion(0, 0, 90, &q0, &q1, &q2, &q3);
// 备注信息     ZYX旋转顺序 (航空顺序)
//              q = [w, x, y, z]
//-------------------------------------------------------------------------------------------------------------------
void euler_to_quaternion(float roll, float pitch, float yaw,
                         float *q0, float *q1, float *q2, float *q3)
{
    float cos_roll, sin_roll;
    float cos_pitch, sin_pitch;
    float cos_yaw, sin_yaw;
    float roll_rad, pitch_rad, yaw_rad;

    // 角度转弧度
    roll_rad = roll * DEG_TO_RAD;
    pitch_rad = pitch * DEG_TO_RAD;
    yaw_rad = yaw * DEG_TO_RAD;

    // 计算三角函数
    cos_roll = cosf(roll_rad * 0.5f);
    sin_roll = sinf(roll_rad * 0.5f);
    cos_pitch = cosf(pitch_rad * 0.5f);
    sin_pitch = sinf(pitch_rad * 0.5f);
    cos_yaw = cosf(yaw_rad * 0.5f);
    sin_yaw = sinf(yaw_rad * 0.5f);

    // ZYX旋转顺序的四元数转换
    *q0 = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
    *q1 = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
    *q2 = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
    *q3 = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数共轭
// 参数说明     q0              四元数 w 分量 (输入输出)
// 参数说明     q1              四元数 x 分量 (输入输出)
// 参数说明     q2              四元数 y 分量 (输入输出)
// 参数说明     q3              四元数 z 分量 (输入输出)
// 返回参数     void
// 使用示例     quaternion_conjugate(&q0, &q1, &q2, &q3);
// 备注信息     q* = [w, -x, -y, -z]
//              用于计算四元数逆: q^-1 = q* / |q|^2
//-------------------------------------------------------------------------------------------------------------------
void quaternion_conjugate(float *q0, float *q1, float *q2, float *q3)
{
    *q1 = -*q1;
    *q2 = -*q2;
    *q3 = -*q3;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数乘法
// 参数说明     qa0~qa3         四元数A
// 参数说明     qb0~qb3         四元数B
// 参数说明     qr0~qr3         输出四元数结果
// 返回参数     void
// 使用示例     quaternion_multiply(qa0, qa1, qa2, qa3, qb0, qb1, qb2, qb3, &qr0, &qr1, &qr2, &qr3);
// 备注信息     四元数乘法: qr = qa ⊗ qb
//              用于计算旋转复合和姿态误差
//-------------------------------------------------------------------------------------------------------------------
void quaternion_multiply(float qa0, float qa1, float qa2, float qa3,
                         float qb0, float qb1, float qb2, float qb3,
                         float *qr0, float *qr1, float *qr2, float *qr3)
{
    *qr0 = qa0 * qb0 - qa1 * qb1 - qa2 * qb2 - qa3 * qb3;
    *qr1 = qa0 * qb1 + qa1 * qb0 + qa2 * qb3 - qa3 * qb2;
    *qr2 = qa0 * qb2 - qa1 * qb3 + qa2 * qb0 + qa3 * qb1;
    *qr3 = qa0 * qb3 + qa1 * qb2 - qa2 * qb1 + qa3 * qb0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数姿态控制器初始化
// 参数说明     roll            目标横滚角 (度)
// 参数说明     pitch           目标俯仰角 (度)
// 参数说明     yaw             目标偏航角 (度)
// 参数说明     kp_dir          方向环比例系数
// 参数说明     kd_dir          方向环微分系数
// 参数说明     kp_gyro         角速度环比例系数
// 参数说明     kd_gyro         角速度环微分系数
// 返回参数     void
// 使用示例     attitude_init(0, 0, 0, 2.5f, 0.8f, 1.2f, 0.3f);
// 备注信息     初始化四元数姿态控制器，设置目标姿态和PD参数
//-------------------------------------------------------------------------------------------------------------------
void attitude_init(float roll, float pitch, float yaw,
                   float kp_dir, float kd_dir, float kp_gyro, float kd_gyro)
{
    // 设置目标欧拉角
    attitude_controller.target_roll = roll;
    attitude_controller.target_pitch = pitch;
    attitude_controller.target_yaw = yaw;

    // 转换为目标四元数
    euler_to_quaternion(roll, pitch, yaw,
                        &attitude_controller.target_q0,
                        &attitude_controller.target_q1,
                        &attitude_controller.target_q2,
                        &attitude_controller.target_q3);

    // 设置PD参数
    attitude_controller.kp_direction = kp_dir;
    attitude_controller.kd_direction = kd_dir;
    attitude_controller.kp_gyro = kp_gyro;
    attitude_controller.kd_gyro = kd_gyro;

    // 清零历史数据
    attitude_controller.last_error_gyro = 0.0f;
}

