/**
 * @file    motor.c
 * @brief   双路直流电机驱动实现文件
 * @details 实现左右两路直流电机的正反转控制、PWM调速，以及风扇控制功能
 * @author  Elec3-5adc Team
 * @date    2026-01-04
 */

// ==============================================================================
// 头文件包含
// ==============================================================================
#include "motor.h"

// ==============================================================================
// 全局变量定义
// ==============================================================================
static int8 duty = 0;          // 电机占空比变量 (用于测试)
static int8 dir = 1;           // 电机方向变量 (用于测试, 1:增加, 0:减少)

// ==============================================================================
// 函数实现
// ==============================================================================

/**
 * @brief  电机GPIO和PWM初始化
 * @details 初始化左右电机和风扇的方向控制引脚及PWM通道
 *          - 左电机: DIR_L(P32) + PWM_L(P33, PWMA_CH4N)
 *          - 右电机: DIR_R(P76) + PWM_R(P75, PWMB_CH2)
 *          - 风扇: FAN_IO(P67) + FAN_PWM(P65, PWMA_CH3N)
 *          - PWM频率统一设置为17kHz，初始占空比为0%
 * @param  None
 * @retval None
 */
void Motor_Init(void)
{
    // 左电机初始化
    gpio_init(DIR_L, GPO, GPIO_LOW, GPO_PUSH_PULL);    // 方向引脚初始化为推挽输出，默认低电平
    pwm_init(PWM_L, MOTOR_PWM_FREQ, 0);                 // PWM通道初始化，频率17kHz，占空比0

    // 右电机初始化
    gpio_init(DIR_R, GPO, GPIO_LOW, GPO_PUSH_PULL);    // 方向引脚初始化为推挽输出，默认低电平
    pwm_init(PWM_R, MOTOR_PWM_FREQ, 0);                 // PWM通道初始化，频率17kHz，占空比0

    // 风扇初始化
    gpio_init(FAN_IO, GPO, GPIO_LOW, GPO_PUSH_PULL);   // 使能引脚初始化为推挽输出，默认低电平
    pwm_init(FAN_PWM, MOTOR_PWM_FREQ, 0);               // PWM通道初始化，频率17kHz，占空比0
}

/**
 * @brief  电机测试函数
 * @details 设置左右电机以固定占空比正转，用于验证电机硬件连接和驱动功能
 *          - 左电机: DIR_HIGH + 7%占空比
 *          - 右电机: DIR_LOW + 7%占空比
 * @note   注释代码为占空比自动变化的测试程序，可根据需要启用
 * @param  None
 * @retval None
 */
void Motor_Test(void)
{
//    // 左电机正转测试
//    gpio_set_level(DIR_L, GPIO_HIGH);                   // 方向引脚置高
//    pwm_set_duty(PWM_L, 7 * (PWM_DUTY_MAX / 100));      // 设置占空比为7%

//    // 右电机正转测试
//    gpio_set_level(DIR_R, GPIO_LOW);                    // 方向引脚置低
//    pwm_set_duty(PWM_R, 7 * (PWM_DUTY_MAX / 100));      // 设置占空比为7%

    // 以下为占空比自动变化的测试代码（已注释）
    // 根据dir变量自动增减占空比，实现电机加减速测试
    
    if(duty >= 0)                                       // 正转
    {
//        gpio_set_level(DIR_L, GPIO_LOW);
//        pwm_set_duty(PWM_L, duty * (PWM_DUTY_MAX / 100));

        gpio_set_level(DIR_R, GPIO_HIGH);
        pwm_set_duty(PWM_R, duty * (PWM_DUTY_MAX / 100));
    }
    else                                                // 反转
    {
//        gpio_set_level(DIR_L, GPIO_HIGH);
//        pwm_set_duty(PWM_L, (-duty) * (PWM_DUTY_MAX / 100));

        gpio_set_level(DIR_R, GPIO_LOW);
        pwm_set_duty(PWM_R, (-duty) * (PWM_DUTY_MAX / 100));
    }
    if(dir)                                             // 根据方向判断增加还是减少（增量式编码器参考）
    {
        duty++;                                         // 占空比增加
        if(duty >= MAX_DUTY)                            // 达到最大值
            dir = 0;                                    // 改变标志位
    }
    else
    {
        duty--;                                         // 占空比减少
        if(duty <= -MAX_DUTY)                           // 达到最小值
            dir = 1;                                    // 改变标志位
    }
    system_delay_ms(100);
    
}

/**
 * @brief  设置左电机正转
 * @details 设置左电机方向为正转(DIR_HIGH)，并设置PWM占空比
 * @param  pwm PWM占空比百分比 (范围: 0-100)
 * @retval None
 */
void Motor_LeftForward(uint8 pwm)
{
    gpio_set_level(DIR_L, GPIO_HIGH);                   // 方向引脚置高，正转方向
    pwm_set_duty(PWM_L, pwm * (PWM_DUTY_MAX / 100));    // 设置PWM占空比
}

/**
 * @brief  设置右电机正转
 * @details 设置右电机方向为正转(DIR_LOW)，并设置PWM占空比
 * @param  pwm PWM占空比百分比 (范围: 0-100)
 * @retval None
 */
void Motor_RightForward(uint8 pwm)
{
    gpio_set_level(DIR_R, GPIO_HIGH);                    // 方向引脚置低，正转方向
    pwm_set_duty(PWM_R, pwm * (PWM_DUTY_MAX / 100));    // 设置PWM占空比
}

/**
 * @brief  设置左电机反转
 * @details 设置左电机方向为反转(DIR_LOW)，并设置PWM占空比
 * @param  pwm PWM占空比百分比 (范围: 0-100)
 * @retval None
 */
void Motor_LeftBackward(uint8 pwm)
{
    gpio_set_level(DIR_L, GPIO_LOW);                    // 方向引脚置低，反转方向
    pwm_set_duty(PWM_L, pwm * (PWM_DUTY_MAX / 100));    // 设置PWM占空比
}

/**
 * @brief  设置右电机反转
 * @details 设置右电机方向为反转(DIR_HIGH)，并设置PWM占空比
 * @param  pwm PWM占空比百分比 (范围: 0-100)
 * @retval None
 */
void Motor_RightBackward(uint8 pwm)
{
    gpio_set_level(DIR_R, GPIO_LOW);                   // 方向引脚置高，反转方向
    pwm_set_duty(PWM_R, pwm * (PWM_DUTY_MAX / 100));    // 设置PWM占空比
}

/**
 * @brief  设置风扇转速
 * @details 设置风扇使能(FAN_IO_HIGH)并调整PWM占空比控制转速
 * @param  pwm PWM占空比百分比 (范围: 0-100)
 * @retval None
 */
void Motor_FanSet(uint8 pwm)
{
    gpio_set_level(FAN_IO, GPIO_HIGH);                  // 风扇使能引脚置高
    pwm_set_duty(FAN_PWM, pwm * (PWM_DUTY_MAX / 100));  // 设置PWM占空比控制转速
}
