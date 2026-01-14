/**
 * @file    motor.h
 * @brief   双路直流电机驱动头文件
 * @details 支持左右两路直流电机的正反转控制和PWM调速，以及一路风扇控制
 * @author  Elec3-5adc Team
 * @date    2026-01-04
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

// ==============================================================================
// 头文件包含
// ==============================================================================
#include "zf_common_headfile.h"

// ==============================================================================
// 硬件引脚定义
// ==============================================================================
#define DIR_L               (IO_P67)              // 左电机方向控制引脚
#define PWM_L               (PWMA_CH3N_P65)      // 左电机PWM输出引脚

#define DIR_R               (IO_P76)              // 右电机方向控制引脚
#define PWM_R               (PWMB_CH2_P75)       // 右电机PWM输出引脚

#define FAN_IO              (IO_P32)              // 风扇使能控制引脚
#define FAN_PWM             (PWMA_CH4N_P33)      // 风扇PWM输出引脚

// ==============================================================================
// 宏定义
// ==============================================================================
#define MAX_DUTY            (10)                  // 最大占空比 10%
#define MOTOR_PWM_FREQ      (17000)              // 电机PWM频率 17kHz

// ==============================================================================
// 函数声明
// ==============================================================================

/**
 * @brief  电机GPIO和PWM初始化
 * @details 初始化左右电机和风扇的方向控制引脚及PWM通道
 *          - PWM频率：17kHz
 *          - 初始占空比：0%
 * @param  None
 * @retval None
 */
void Motor_Init(void);

/**
 * @brief  电机测试函数
 * @details 设置固定的正转状态用于测试电机功能
 * @param  None
 * @retval None
 */
void Motor_Test(void);

/**
 * @brief  设置左电机正转
 * @details 设置左电机方向为正转，并设置PWM占空比
 * @param  pwm PWM占空比百分比 (0-100)
 * @retval None
 */
void Motor_LeftForward(uint8 pwm);

/**
 * @brief  设置右电机正转
 * @details 设置右电机方向为正转，并设置PWM占空比
 * @param  pwm PWM占空比百分比 (0-100)
 * @retval None
 */
void Motor_RightForward(uint8 pwm);

/**
 * @brief  设置左电机反转
 * @details 设置左电机方向为反转，并设置PWM占空比
 * @param  pwm PWM占空比百分比 (0-100)
 * @retval None
 */
void Motor_LeftBackward(uint8 pwm);

/**
 * @brief  设置右电机反转
 * @details 设置右电机方向为反转，并设置PWM占空比
 * @param  pwm PWM占空比百分比 (0-100)
 * @retval None
 */
void Motor_RightBackward(uint8 pwm);

/**
 * @brief  设置风扇转速
 * @details 设置风扇使能并调整PWM占空比控制转速
 * @param  pwm PWM占空比百分比 (0-100)
 * @retval None
 */
void Motor_FanSet(uint8 pwm);

#endif /* _MOTOR_H_ */
