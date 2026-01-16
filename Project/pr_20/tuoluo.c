#include "tuoluo.h"




//角度
float gyro_z[2]={0}, next_gyro_z=0;			//(被测)Z轴角速度
float angle_ringR=0;

float null_drift_z=0;	//零漂
float avl_gyro_z=0;   //测量





/****************角度获取****************/

void tuoluo_init(void)
{
//	icm42688_init();
//	imu660ra_init();
	icm20602_init();

	
}

void gyroscope_get_gyro(void)
{
	
	
	icm20602_get_gyro();

	gyro_z[0] = 0.9*(-icm20602_gyro_z)+0.1*gyro_z[1];
//	
	
	
	
//	
//	imu660ra_get_gyro();
//	gyro_z[0] = 0.9*(-imu660ra_gyro_z)+0.1*gyro_z[1];
//	
	
	
	gyro_z[1] = gyro_z[0];
	next_gyro_z = imu660ra_gyro_transition(gyro_z[0]);
	
	avl_gyro_z = next_gyro_z - null_drift_z;
	
	
	angle_get();

}

void angle_get(void)
{
	angle_ringR+=0.005*avl_gyro_z;
}

void angle_clear(void)
{
	angle_ringR=0;
}


int8 null_drift_calculate(void)
{
	static int16 cnt_null=0;
	static float temp=0;
	static int8 ret=0;
	
	//采集数据
//	
	
	temp += next_gyro_z;
		
	cnt_null++;
	if(cnt_null>=200)
	{
		null_drift_z = temp/200;
		ret = 1;
		angle_clear();
	}
	
	return ret;
}






