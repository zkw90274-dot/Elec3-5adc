#ifndef __MOTOR_H
#define __MOTOR_H

#include "bsp_system.h"

#define DIR_L P50
#define DIR_R P51
#define PWM_L (PWMB_CH4_P53)
#define PWM_R (PWMB_CH3_P52)

void motor_init(void);
void motor_L(int16 duty);
void motor_R(int16 duty);



void fuya_init(void);
void fuya_duty(int16 duty);




#endif