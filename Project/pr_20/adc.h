#ifndef __ADC_H
#define __ADC_H


#include "bsp_system.h"

#define ADC_Sample_Num 7

//新版
#define ADC_L_CH 			ADC_CH0_P10
#define ADC_LM_CH     ADC_CH8_P00
#define ADC_M_CH      ADC_CH9_P01
#define ADC_RM_CH     ADC_CH13_P05
#define ADC_R_CH      ADC_CH14_P06

#define ADC_V ADC_CH2_P12

//旧版
//#define ADC_L_CH 			ADC_CH4_P14
//#define ADC_LM_CH     ADC_CH2_P12
//#define ADC_M_CH      ADC_CH0_P10
//#define ADC_RM_CH     ADC_CH8_P00
//#define ADC_R_CH      ADC_CH9_P01


//ADC
extern float ADC_temp[5];
extern float L,LM,M,RM,R;

extern float adc_v,adc_v_2,adc_v_3,adc_v_4;
extern float adc_start;

//算法
extern uint16 ADC_original[5][ADC_Sample_Num];




void my_adc_init(void);
//电磁采样
uint16 adc_sample_a(adc_channel_enum ch);		
uint16 adc_sample_b(adc_channel_enum ch);

uint16 adc_sample(adc_channel_enum ch);			


uint16 adc_mid_sample(adc_channel_enum ch);//三次取中值



void siai_adc_all_sample(void);													//电磁采样
void adc_normalizing(void);											//归一化

//电压采样
float adc_v_sample(float v_next);
float adc_v_sample_2(float v_next);
float adc_v_sample_3(float v_next);
float adc_v_sample_4(float v_next);

#endif
