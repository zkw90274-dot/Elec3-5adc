#include "pid.h"

// 全局变量定义
pid_param_t pid_motor_left;     // 左电机PID控制器
pid_param_t pid_motor_right;    // 右电机PID控制器
pid_param_t pid_SDSD;           // 差比和差PID控制器

// PD控制器全局变量
pd_param_t pd_direction;        // 方向环PD参数
pd_param_t pd_angle;            // 角度环PD参数
pd_param_t pd_gyro;             // 角速度环PD参数

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
// 函数简介     清除PID数据
// 参数说明     pid             PID结构体指针
// 返回参数     void
// 使用示例     pid_clear(&pid_motor_left);
//-------------------------------------------------------------------------------------------------------------------
void pid_clear(pid_param_t *pid)
{
    pid->error = 0;
    pid->last_error = 0;
    pid->prev_error = 0;
    pid->integrator = 0;
    pid->out = 0;
    pid->out_p = 0;
    pid->out_i = 0;
    pid->out_d = 0;
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
// 函数简介     PD方向环 - 电磁位置误差转期望角速度
// 参数说明     err_position    电磁位置误差
// 返回参数     float           电机差速修正值
// 使用示例     float correction = pd_direction_loop(adc_error);
// 备注信息     两级PD控制：位置误差->期望角速度->角速度误差->电机差速
//-------------------------------------------------------------------------------------------------------------------
float pd_direction_loop(float err_position)
{
    static float err_position_history[4] = {0};
    static float err_gyro_history[2] = {0};
    float expect_gyro;
    float err_gyro;
    float correction;

    // 误差历史更新
    err_position_history[2] = err_position_history[1];
    err_position_history[1] = err_position_history[0];
    err_position_history[0] = err_position;

    // 第一级PD：位置误差 -> 期望角速度
    expect_gyro = pd_direction.kp * err_position_history[0] +
                  pd_direction.kd * (err_position_history[0] - err_position_history[1]);

    // 角速度期望限幅
    if(expect_gyro > 100.0f)
        expect_gyro = 100.0f;
    else if(expect_gyro < -100.0f)
        expect_gyro = -100.0f;

    // 第二级PD：角速度误差 -> 电机差速修正值
    err_gyro = expect_gyro - imu.gyro_z;
    err_gyro_history[1] = err_gyro_history[0];
    err_gyro_history[0] = err_gyro;

    correction = pd_direction.kp_gyro * err_gyro_history[0] +
                 pd_direction.kd_gyro * (err_gyro_history[0] - err_gyro_history[1]);

    // 输出限幅
    if(correction > 100.0f)
        correction = 100.0f;
    else if(correction < -100.0f)
        correction = -100.0f;

    return correction;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PD角度环 - 角度误差转期望角速度
// 参数说明     current_angle   当前角度(如yaw)
// 参数说明     target_angle    目标角度
// 返回参数     float           期望角速度
// 使用示例     float expect_gyro = pd_angle_loop(current_yaw, target_yaw);
// 备注信息     用于转弯控制：角度误差->PD计算->期望角速度
//-------------------------------------------------------------------------------------------------------------------
float pd_angle_loop(float current_angle, float target_angle)
{
    static float err_angle_history[3] = {0};
    float err_angle = 0;
    float expect_gyro = 0;

    // 角度误差计算(处理角度跳变)
    err_angle = target_angle - current_angle;
    if(err_angle > 180.0f)
        err_angle -= 360.0f;
    else if(err_angle < -180.0f)
        err_angle += 360.0f;

    // 误差历史更新
    err_angle_history[2] = err_angle_history[1];
    err_angle_history[1] = err_angle_history[0];
    err_angle_history[0] = err_angle;

    // PD计算：角度误差 -> 期望角速度
    expect_gyro = pd_angle.kp * err_angle_history[0] +
                  pd_angle.kd * (err_angle_history[1] - err_angle_history[2]);

    // 输出限幅
    if(expect_gyro > 100.0f)
        expect_gyro = 100.0f;
    else if(expect_gyro < -100.0f)
        expect_gyro = -100.0f;

    return expect_gyro;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PD角速度环 - 角速度误差转电机控制值
// 参数说明     expect_gyro     期望角速度
// 参数说明     actual_gyro     实际角速度
// 返回参数     int16           电机控制值
// 使用示例     int16 output = pd_gyro_loop(expect_gyro, actual_z_gyro);
// 备注信息     用于角速度闭环控制：角速度误差->PD计算->电机控制量
//-------------------------------------------------------------------------------------------------------------------
int16 pd_gyro_loop(float expect_gyro, float actual_gyro)
{
    static float err_gyro_history[2] = {0};
    float err_gyro = 0;
    int16 output = 0;

    // 角速度误差计算
    err_gyro = expect_gyro - actual_gyro;

    // 误差历史更新
    err_gyro_history[1] = err_gyro_history[0];
    err_gyro_history[0] = err_gyro;

    // PD计算：角速度误差 -> 电机控制值
    output = (int16)(pd_gyro.kp_gyro * err_gyro_history[0] +
                     pd_gyro.kd_gyro * (err_gyro_history[0] - err_gyro_history[1]));

    // 输出限幅
    if(output > 5000)
        output = 5000;
    else if(output < -5000)
        output = -5000;

    return output;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PD方向环+角速度环组合控制 (推荐用于循迹)
// 参数说明     err_position    位置误差(ADC归一化值-中心值)
// 返回参数     float           电机差速修正值
// 使用示例     float correction = pd_direction_gyro_loop(adc_error);
// 备注信息     位置误差->期望角速度->角速度闭环->电机差速修正值
//              控制流程:
//              1. 位置误差经PD计算得到期望角速度
//              2. 期望角速度与实际角速度比较
//              3. 角速度误差经PD计算得到电机差速修正值
//              4. 双环结构提供快速响应和良好阻尼
//-------------------------------------------------------------------------------------------------------------------
float pd_direction_gyro_loop(float err_position)
{
    static float err_position_history[4] = {0};
    static float err_gyro_history[2] = {0};
    float expect_gyro;
    float err_gyro;
    float correction;

    // ========== 第一级：PD方向环 ==========
    // 误差历史更新
    err_position_history[2] = err_position_history[1];
    err_position_history[1] = err_position_history[0];
    err_position_history[0] = err_position;

    // PD计算：位置误差 -> 期望角速度
    expect_gyro = pd_direction.kp * err_position_history[0] +
                  pd_direction.kd * (err_position_history[0] - err_position_history[1]);

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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除PD控制器历史误差
// 参数说明     void
// 返回参数     void
// 使用示例     pd_clear_history();
// 备注信息     清除所有PD控制器的误差历史，用于重启控制或切换模式
//              注意：此函数仅清除误差历史，不清除PD参数
//-------------------------------------------------------------------------------------------------------------------
void pd_clear_history(void)
{
    // 注意：静态变量无法在外部直接清零
    // 这是一个示例函数，实际使用时需要在各PD函数内部添加清除逻辑
    // 或者重启控制器来清除历史误差
}
