#include "motor.h"







void motor_init(void)
{
	pwm_init(PWMB_CH3_P52,17000,0);
	pwm_init(PWMB_CH4_P53,17000,0);
	
	gpio_init(IO_P50,GPO,1,GPO_PUSH_PULL);
	gpio_init(IO_P51,GPO,1,GPO_PUSH_PULL);
}

void motor_L(int16 duty)
{
	float temp;
//	temp = 1.15*adc_v-1.65;
	
	duty = duty/adc_v*12;
	
	if(duty>9900){duty=9900;}
	if(duty<-9900){duty=-9900;}
	
	if(duty>0)
	{
		DIR_L=1;
		pwm_set_duty(PWM_L,duty);
	}
	else
	{
		DIR_L=0;
		pwm_set_duty(PWM_L,-duty);
	}
}

void motor_R(int16 duty)
{
	float temp;
//	temp = 1.15*adc_v-1.65;
	
	duty = duty/adc_v*12;


	if(duty>9900){duty= 9900;}
	if(duty<-9900){duty=-9900;}
	
	if(duty>0)
	{
		DIR_R=1;
		pwm_set_duty(PWM_R,duty);
	}
	else
	{
		DIR_R=0;
		pwm_set_duty(PWM_R,-duty);
	}
}














void fuya_init(void)
{
		pwm_init(PWMA_CH3P_P24,400,0);
		
}


void fuya_duty(int16 duty)
{
	if(duty>8000){duty=8000;}
	
	if(duty>0)
	{
		pwm_set_duty(PWMA_CH3P_P24,duty);
	}
	else
	{
		pwm_set_duty(PWMA_CH3P_P24,-duty);
	}


}





