#ifndef __CLOSED_LOOP_H
#define __CLOSED_LOOP_H


#include "bsp_system.h"

	/*********PID*********/
extern struct PID
{
	float Kp;
	float Ki;
	float Kd;
	float Kp_gyro;
	float Ki_gyro;
	float Kd_gyro;
};

	/*************变量*************/

//速度环
extern int16 err_speed;				//误差
extern float out_L,out_R;			//输出


//方向环
extern float expect_gyro;			//误差
extern float correct_L;				//输出


//角度环
extern float expect_gyro_angles;
extern float out_angle;

//角速度环
extern int16 out_gyro;

	/*************函数*************/

void direction_return(float err);													//方向环
void loop_angle(float next_angle,float target_angle);		//角度环
void gyro_loop(float gyro_expect);												//角速度环

void loop_speed(void);																		//速度环
void loop_speed_LR(int16 speed_L_t,int16 speed_R_t);





#endif
