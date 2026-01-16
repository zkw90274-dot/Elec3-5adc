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
// 参数说明     encoder_left    左编码器值
// 参数说明     encoder_right   右编码器值
// 返回参数     void
// 使用示例     motor_pid_control(encoder_left, encoder_right);
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

    input_SDSD = SDSD_calculate(&SDSD);
    output_SDSD = pid_calc_position(&pid_SDSD, input_SDSD);
    final_input_left = encoder_left + output_SDSD;
    final_input_right = encoder_right - output_SDSD;
    output_left = pid_calc_increment(&pid_motor_left, final_input_left);
    output_right = pid_calc_increment(&pid_motor_right, final_input_right);

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
