﻿#include "adc.h"



//1-电磁
uint16 ADC_original[5][ADC_Sample_Num]={0};
uint16 max[5]={0},min[5]={0};
float ADC_temp[5] = {0};
float L=0,LM=0,M=0,RM=0,R=0;

//2-电源电压
float adc_v,adc_v_2,adc_v_3,adc_v_4;
float adc_start =7;   //13.2    10.7


void my_adc_init(void)
{
	adc_init(ADC_L_CH ,ADC_10BIT);		//L
	adc_init(ADC_LM_CH,ADC_10BIT);		//LM	
	adc_init(ADC_M_CH ,ADC_10BIT);		//M	
	adc_init(ADC_RM_CH,ADC_10BIT);		//RM	
	adc_init(ADC_R_CH ,ADC_10BIT);		//R	

	adc_init(ADC_V ,ADC_10BIT);		//R
	
}

/************电磁采样************/
uint16 adc_sample(adc_channel_enum ch)
{
	uint16 temp,sum=0;
	uint16 max,min;
	uint8 i;
	
	temp = adc_convert(ch);
	max = temp;
	min = temp;
	sum += temp;

	for(i=0;i<10;i++)
	{
		temp = adc_convert(ch);
		if(max<temp){max = temp;}
		if(min>temp){min = temp;}
		sum += temp;
	}
	
	temp = (sum-max-min) / (11-2);
	
	return temp;
}

uint16 adc_sample_b(adc_channel_enum ch)
{
	uint16 temp,sum=0;
	uint16 max,min;
	uint8 i;
	
	temp = adc_mid_sample(ch);
	max = temp;
	min = temp;
	sum += temp;

	for(i=1;i<=7;i++)
	{
		temp = adc_mid_sample(ch);
		if(max<temp){max = temp;}
		if(min>temp){min = temp;}
		sum += temp;
	}
	
	temp = (sum-max-min) / (8-2);
	
	return temp;
}


uint16 adc_sample_a(adc_channel_enum ch)
{
	uint8 i;
	uint16 arr[7];
	
	for(i=0;i<ADC_Sample_Num;i++)
	{
		arr[i] = adc_convert(ch);
	}
	
	arr[0] = sort_seven(arr);	
	return arr[0];
}


uint16 adc_mid_sample(adc_channel_enum ch)//三次取中值
{
    uint16 i,j,k,tmp;
    //sample 3 times
    i = adc_convert(ch);
    j = adc_convert(ch);
    k = adc_convert(ch);
    //  select mid value
    if(i > j){
        tmp = i;
        i = j;
        j = tmp;
    }
    if(k > j){
        tmp = j;
    }
    else if(k > i){
        tmp = k;
    }
    else{
        tmp = i;
    }
    return tmp;
}













void siai_adc_all_sample(void)
{
	ADC_temp[0] = adc_sample(ADC_CH0_P10 );
	ADC_temp[1] = adc_sample(ADC_CH8_P00 );
	ADC_temp[2] = adc_sample(ADC_CH9_P01 );
	ADC_temp[3] = adc_sample(ADC_CH13_P05);
	ADC_temp[4] = adc_sample(ADC_CH14_P06);

}



//归一化
void adc_normalizing(void)
{
	//归一化
//	L  = 100*(ADC_temp[0]-0)/(600-0);
//  LM = 100*(ADC_temp[1]-0)/(600-0);
//	M  = 100*(ADC_temp[2]-0)/(600-0);
//  RM = 100*(ADC_temp[3]-0)/(600-0);
//	R  = 100*(ADC_temp[4]-0)/(600-0);
	
	
 
	L  = 100*(ADC_temp[0]-0)/(600-0);
  LM = 100*(ADC_temp[1]-0)/(600-0);
	M  = 100*(ADC_temp[2]-0)/(600-0);
	RM = 100*(ADC_temp[3]-0)/(600-0);
	R  = 100*(ADC_temp[4]-0)/(600-0);
	
	
	

	//输入限幅
	if(L >100){L =100;}
	if(LM>100){LM=100;}
	if(M >100){M =100;}
	if(RM>100){RM=100;}
	if(R >100){R =100;}
}





//电压采集
float adc_v_sample(float v_next)
{
	static float v,last_v=0;
	v = v_next;
	v = v*0.014648*4;
	
	
	v = 0.1*v + 0.90*last_v;
	last_v = v;
	
	return v;
}

float adc_v_sample_2(float v_next)
{
	static float v,last_v=0;
	v = v_next;
	v = v*0.038671;
	
	
	v = 0.5*v + 0.5*last_v;
	last_v = v;
	
	return v;
}
	
float adc_v_sample_3(float v_next)
{
	static float v,last_v=0;
	v = v_next;
	v = v*0.038671;
	
	
	v = 0.1*v + 0.9*last_v;
	last_v = v;
	
	return v;
}

float adc_v_sample_4(float v_next)
{
	static float v,last_v=0;

	v = v_next;

	v = v*0.038671;
	
	
	v = 0.05*v + 0.95*last_v;
	last_v = v;
	
	return v;
}
