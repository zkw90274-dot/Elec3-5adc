//头文件包含
#include "pid.h"



// 全局变量定义
pid_param_t pid_motor_left;     // 左电机PID控制器
pid_param_t pid_motor_right;    // 右电机PID控制器
pid_param_t pid_SDSD;           // 差比和差PID控制器

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID参数初始化
// 参数说明     pid             PID结构体指针
// 参数说明     kp              比例系数
// 参数说明     ki              积分系数
// 参数说明     kd              微分系数
// 返回参数     void
// 使用示例     pid_init(&pid_motor_left, 1.7, 0.22, 0.2);
// 备注信息     初始化PID控制器的所有参数和状态变量
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
// 备注信息     动态调整PID参数，不影响其他状态变量
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
// 备注信息     设置PID控制器的目标值(注：函数内部对target取负)
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
// 备注信息     清除PID的所有误差和输出累计值，用于重启控制或切换模式
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
// 使用示例     float result = constrain_float(value, -100, 100);
// 备注信息     将输入值限制在指定范围内
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
// 使用示例     float output = pid_calc_position(&pid_motor_left, encoder_value);
// 备注信息     位置式PID，输出是绝对控制量
//-------------------------------------------------------------------------------------------------------------------
float pid_calc_position(pid_param_t *pid, float actual)
{
    pid->actual = actual;
    pid->error = pid->target - actual;

    // 积分累计
    pid->integrator += pid->error;

    // 积分限幅
    pid->integrator = constrain_float(pid->integrator, -pid->integral_max, pid->integral_max);

    // 计算PID各项
    pid->out_p = pid->kp * pid->error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (pid->error - pid->last_error);

    // 更新误差
    pid->last_error = pid->error;

    // 计算总输出
    pid->out = pid->out_p + pid->out_i + pid->out_d;

    // 输出限幅
    pid->out = constrain_float(pid->out, pid->output_min, pid->output_max);

    return pid->out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     增量式PID计算
// 参数说明     pid             PID结构体指针
// 参数说明     actual          当前实际值
// 返回参数     float           PID输出值
// 使用示例     float output = pid_calc_increment(&pid_motor_left, encoder_value);
// 备注信息     增量式PID，输出是控制量的增量，需要累加到之前的输出
//-------------------------------------------------------------------------------------------------------------------
float pid_calc_increment(pid_param_t *pid, float actual)
{
    float increment;

    pid->actual = actual;
    pid->error = pid->target - actual;

    // 计算增量式PID各项
    // Δu(k) = Kp[e(k)-e(k-1)] + Ki*e(k) + Kd[e(k)-2e(k-1)+e(k-2)]
    pid->out_p = pid->kp * (pid->error - pid->last_error);
    pid->out_i = pid->ki * pid->error;
    pid->out_d = pid->kd * (pid->error - 2 * pid->last_error + pid->prev_error);

    // 计算增量
    increment = pid->out_p + pid->out_i + pid->out_d;

    // 更新误差
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;

    // 累加输出
    pid->out += increment;

    // 输出限幅
    pid->out = constrain_float(pid->out, pid->output_min, pid->output_max);

    return pid->out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机PID控制函数
// 参数说明     encoder_left    左编码器值
// 参数说明     encoder_right   右编码器值
// 返回参数     void
// 使用示例     motor_pid_control(encoder_left, encoder_right);
// 备注信息     在定时中断中调用，根据编码器值计算PWM输出并控制电机
//              首先使用差比和差进行位置式PID计算，然后使用增量式PID控制电机速度
//-------------------------------------------------------------------------------------------------------------------
void motor_pid_control(int16 encoder_left, int16 encoder_right)
{
    float output_left = 0;
    float output_right = 0;
    float input_SDSD = 0;
    float output_SDSD = 0;
    float final_input_left = 0;
    float final_input_right = 0;

    uint8 pwm_left = 0;
    uint8 pwm_right = 0;

    // 计算差比和差值
    input_SDSD = SDSD_calculate(&SDSD);
    output_SDSD = pid_calc_position(&pid_SDSD, input_SDSD);

    // 计算差速后的输入值
    final_input_left = encoder_left + output_SDSD;
    final_input_right = encoder_right - output_SDSD;

    // 计算增量式PID输出（已经融合了差速，-100~100限幅）
    output_left = pid_calc_increment(&pid_motor_left, final_input_left);
    output_right = pid_calc_increment(&pid_motor_right, final_input_right);

    // ===== 左电机控制（带限幅保护）=====
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

    // ===== 右电机控制（带限幅保护）=====
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
