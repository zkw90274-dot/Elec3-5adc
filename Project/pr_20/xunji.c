#include "xunji.h"

//P I D //P D								 // 160 180			16  20
struct PID pid_motor_straight={200,0,220  ,18,0,23};			//循迹
struct PID pid_motor_turn=		{180,0,60   ,16,0,20};	    //12 3
struct PID pid_motor_ringR=		{750,0,45   ,16,0,20};

struct PID pid_loop_speed_start=		{10,3};										//速度环
struct PID pid_loop_speed_run=	  	{20,5};										//速度环

struct PID pid_loop_angle_ring=		{50,0,0   ,16,0,20};				//角度环

struct PID pid_loop_gyro=		  {0,0,0   ,16,0,20};					//角速度环

float limit_gyro = 800;


//误差
float err_dianci=0;
float e3,e1;

//2-循迹(速度)
struct PID pid_motor_run;
struct PID pid_loop_speed = {10,3};										//速度环
struct PID pid_loop_angle;														//角度环
struct PID pid_loop_angle_start={16,0,0   ,17,0,20};				//角度环



int8  element = 0;
int16 speed_target = 0;
int16 speed_straight = 195;
int16 speed_turn = 550;
int16 speed_ringR = 190;
float gyro_adv_out = 650;


//环内切1
int16 distance_ringR_1 = 2300;
int16 gyro_ring_in = 550;
int16 gyro_ring_middle =750;
int16 gyro_ring_out = 500;

//环循迹2
int16 distance_ringR_trace = 4000;
int16 gyro_ring_in_trace = 600;
int16 gyto_ring_out_trace = 650;


float limit_gyro_t=850;

float angle_start=70;

//5-元素执行
int16 ring_flag_ing=0,ringR_flag_task=1,ringR_flag_execute=1;
int32 distance_ringR=0;
int16 distance_ringR_target=2300;
int8 turn_flag=0;

float ring_L_L =55;
float ring_L_M =80;
							
float ring_R_M =80;
float ring_R_R =55;



float A_=45;   //45
float B_=1;  //2
float C_=0.15; //0.1
float D_=7;    //7
float K_=1;
float F_=0.61;





float A_dianci=0,B_dianci=0,C_dianci=0,all_dianci=0;

float k=0;

void dianci_xunji(void)
{
	static int times=0,turn_flag_cnt=0;
	static float LRM_fabs;
	static float speed_temp;
	
	static float a=30;
	static float b=60;
	static float c=0,d=0,temp=0;

	//1-电磁滤波输入
	siai_adc_all_sample();							//中间平均滤波
	adc_normalizing();									//电磁归一化
	
	
	//2-err

		LRM_fabs = RM-LM;
		if(LRM_fabs<0){LRM_fabs = -LRM_fabs;}
		
		if(times == 0)
		{
			temp = 0.6/(b-a);
			d = -a*temp;
			c = temp;
			times=1;
		}
		if(M>b){k=0.8;}
		else if(M<a){k=0.2;}
		else{k = c*M+d+0.2;}
		
		A_dianci =       K_*(R-L);
		B_dianci = (1-k)*B_*(RM - LM);
		C_dianci = (1-k)*C_*LRM_fabs;

		
		all_dianci = (L+R) + D_*M + C_dianci;
		if(all_dianci<250){all_dianci=250;}
		
		err_dianci = A_ * ( A_dianci + B_dianci )
										/ (all_dianci);

	
		
	//3-元素判断
	
	if(M>ring_R_M && R>ring_R_R && ring_flag_ing==0)	//左环岛
	{
		if(Ring_choice == 1){element = 2;}
		else(element = 4);
		
		turn_flag = 4;
	} 
	
	if(M>ring_L_M	&& L>ring_L_L && ring_flag_ing==0)	//右环岛
	{
		if(Ring_choice == 1){element = 1;}
		else(element = 3);			
		turn_flag = 5;
	}
	
	if(L+LM+RM+R<20)									//出界保护
	{element = 9;			turn_flag = 9;}


	
	//4-循迹运行
	switch (element)
	{
		case 0:												/**正常循迹**/
			turn_flag=0;
			pid_motor_run=pid_motor_straight;
			speed_target = speed_straight;	
			
			direction_return(err_dianci);

		  if(correct_L>0)
		  {
		  	loop_speed_LR(speed_target + small_t*correct_L,speed_target - large_t*correct_L);
		  
		  }
		  else
		  {
		  	loop_speed_LR(speed_target + large_t*correct_L,speed_target - small_t*correct_L);
		  }
		     
		    //慢慢降比例
		    
			motor_L(out_L);
	    motor_R(out_R);
			
			if(ring_flag_ing>0)
			{
			 ring_flag_ing--;
			}
			break;
			
				/**环岛任务**/
		case 1:							//右环岛				
			ringR_task();
			ringR_execute();
			break;
			
		
		case 2:							//左环岛
			ringL_task();
			ringL_execute();
			break;
		case 3:
			ringR_task_trace();
			ringR_execute_trace();
			break;
		case 4:
			ringL_task_trace();
			ringL_execute_trace();
			break;
		case 9:
//			speed_target = 0;
//			loop_speed();
			
			motor_L(0);
			motor_R(0);
			break;
		default:break;
		
	}
	
	
}








		/***************元素执行**************/
//环岛状态判断
void ringR_task_trace(void)
{
	switch(ringR_flag_task)
	{
		case 1:													//1准备进岛
			ring_flag_ing =200;
			ringR_flag_execute=1;
			distance_ringR += speed_avl;
			if(distance_ringR>distance_ringR_trace)							//2到达打角点
			{	
				angle_ringR = 0;
				distance_ringR=0;
				ringR_flag_task=2;
				ringR_flag_execute=2;	
				angle_clear();	
			}
			break;
			

		case 2:								
											//1进岛角度积分
		
			if(angle_ringR>40)						//2打角完成
			{
				ringR_flag_task=3;
				ringR_flag_execute=3;
			}
			break;
			
			
		case 3:
											//1岛内角度积分
		
			if(angle_ringR>260)					//2即将到达打角点
			{							//335
				ringR_flag_task=4;
				ringR_flag_execute=4;
			}
			break;
			
			
		case 4:
											//1岛内角度积分
			if(angle_ringR>330)			
			{
				
				ringR_flag_task=5;
				ringR_flag_execute=5;
			}
			break;
		
		case 5:
			
			distance_ringR += speed_avl;
			
			if(distance_ringR>5000)							//2到达打角点
			{	
				angle_clear();
				distance_ringR=0;
				ringR_flag_task=1;
				ringR_flag_execute=1;
				element=0;
				distance_ringR=0;

			}
			
			
		
		default:
			break;
	
	}
}







//环岛任务执行
void ringR_execute_trace(void)
{
	static float expect_gyro;
	switch(ringR_flag_execute){
		case 1:															//1-直线x进岛			
			speed_target=speed_ringR;					
			gyro_loop(0);
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}
			motor_L(out_L);
	    motor_R(out_R);
			break;
		
    case 2:															//2-打角进岛
			speed_target = speed_ringR;			
			gyro_loop(gyro_ring_in_trace);		
		
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}

			motor_L(out_L);
	    motor_R(out_R);
			break;
		
		case 3:															//3-岛内循迹
			speed_target = speed_ringR;
			pid_motor_run = pid_motor_turn;
			limit_gyro =650;
			direction_return(err_dianci);
			if(correct_L>0)
			{
				loop_speed_LR(speed_target + small_t*correct_L,speed_target - large_t*correct_L);

			}
			else
			{
				loop_speed_LR(speed_target + large_t*correct_L,speed_target - small_t*correct_L);
			}			
			motor_L(out_L);
	    motor_R(out_R);
			break;
		
		case 4:															//固定角速度出岛
			speed_target = speed_ringR;			
			gyro_loop(gyto_ring_out_trace);		
		
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}
			motor_L(out_L);
	    motor_R(out_R);
			break;
		
		case 5:															//直线x出岛
			speed_target=speed_straight;					
			gyro_loop(0);
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}			motor_L(out_L);
	    motor_R(out_R);
		
		default:break;
	}
}


















//环岛状态判断
void ringL_task_trace(void)
{
	switch(ringR_flag_task)
	{
		case 1:													//1准备进岛

			ring_flag_ing = 200;
			ringR_flag_execute=1;
			distance_ringR += speed_avl;
			
			if(distance_ringR>distance_ringR_trace)							//2到达打角点
			{	
				angle_ringR = 0;
				distance_ringR=0;
				ringR_flag_task=2;
				ringR_flag_execute=2;	
				angle_clear();	
			}
			break;
			

		case 2:								
											//1进岛角度积分
		
			if(angle_ringR<-40)						//2打角完成
			{
				ringR_flag_task=3;
				ringR_flag_execute=3;
			}
			break;
			
			
		case 3:
											//1岛内角度积分
		
			if(angle_ringR<-260)					//2即将到达打角点
			{							//335
				ringR_flag_task=4;
				ringR_flag_execute=4;
			}
			break;
			
			
		case 4:
											//1岛内角度积分
			if(angle_ringR<-330)			
			{
				
				ringR_flag_task=5;
				ringR_flag_execute=5;
			}
			break;
		
		case 5:
			
			distance_ringR += speed_avl;
			
			if(distance_ringR>5000)							//2到达打角点
			{	
				angle_clear();
				distance_ringR=0;
				ringR_flag_task=1;
				ringR_flag_execute=1;
				element=0;
				distance_ringR=0;
			}
			
			
		
		default:
			break;
	
	}

}

//环岛任务执行
void ringL_execute_trace(void)
{
	static float expect_gyro;
	switch(ringR_flag_execute){
		case 1:															//1-直线x进岛			
			speed_target=speed_ringR;					
			gyro_loop(0);
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}
			motor_L(out_L);
	    motor_R(out_R);
			break;
		
    case 2:															//2-打角进岛
			speed_target = speed_ringR;
			gyro_loop(-gyro_ring_in_trace);		
		
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}
			motor_L(out_L);
	    motor_R(out_R);
			break;
		
		case 3:															//3-岛内循迹
			speed_target = speed_ringR;
			pid_motor_run = pid_motor_turn;
			limit_gyro =650;
			direction_return(err_dianci);
			if(correct_L>0)
			{
				loop_speed_LR(speed_target + small_t*correct_L,speed_target - large_t*correct_L);

			}
			else
			{
				loop_speed_LR(speed_target + large_t*correct_L,speed_target - small_t*correct_L);
			}			
			motor_L(out_L);
	    motor_R(out_R);
			break;
		
		case 4:															//固定角速度出岛
			speed_target = speed_ringR;			
			gyro_loop(-gyto_ring_out_trace);		
		
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}
			motor_L(out_L);
	    motor_R(out_R);
			break;
		
		case 5:															//直线x出岛
			speed_target=speed_straight;					
			gyro_loop(0);
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}			motor_L(out_L);
	    motor_R(out_R);
		
		default:break;
	}
}








//环岛状态判断
void ringR_task(void)
{
	switch(ringR_flag_task)
	{
		case 1:													//1准备进岛
			ring_flag_ing =200;
			ringR_flag_execute=1;
			distance_ringR += speed_avl;
			if(distance_ringR>distance_ringR_1)							//2到达打角点
			{	
				angle_ringR = 0;
				distance_ringR=0;
				ringR_flag_task=2;
				ringR_flag_execute=2;	
				angle_clear();	
			}
			break;
			

		case 2:								
											//1进岛角度积分
		
			if(angle_ringR>50)						//2打角完成
			{
				ringR_flag_task=3;
				ringR_flag_execute=3;
			}
			break;
			
			
		case 3:
											//1岛内角度积分
		
			if(angle_ringR>260)					//2即将到达打角点
			{							//335
				ringR_flag_task=4;
				ringR_flag_execute=4;
			}
			break;
			
			
		case 4:
											//1岛内角度积分
			if(angle_ringR>340)			
			{
				
				ringR_flag_task=5;
				ringR_flag_execute=5;
			}
			break;
		
		case 5:
			
			distance_ringR += speed_avl;
			
			if(distance_ringR>5000)							//2到达打角点
			{	
				angle_clear();
				distance_ringR=0;
				ringR_flag_task=1;
				ringR_flag_execute=1;
				element=0;
				distance_ringR=0;

			}
			
			
		
		default:
			break;
	
	}
}







//环岛任务执行
void ringR_execute(void)
{
	static float expect_gyro;
	switch(ringR_flag_execute){
		case 1:															//1-直线x进岛			
			speed_target=speed_ringR;					
			gyro_loop(0);
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}
			motor_L(out_L);
	    motor_R(out_R);
			break;
		
    case 2:															//2-打角进岛
			speed_target = speed_ringR;			
			gyro_loop(gyro_ring_in);		
		
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}

			motor_L(out_L);
	    motor_R(out_R);
			break;
		
		case 3:															//3-角速度环
			speed_target = speed_ringR;			
			gyro_loop(gyro_ring_middle);		
		
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}

			motor_L(out_L);
	    motor_R(out_R);
			break;
		
		case 4:															//固定角速度出岛
			speed_target = speed_ringR;			
			gyro_loop(gyro_ring_out);		
		
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}
			motor_L(out_L);
	    motor_R(out_R);
			break;
		
		case 5:															//直线x出岛
			speed_target=speed_straight;					
			gyro_loop(0);
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}			motor_L(out_L);
	    motor_R(out_R);
		
		default:break;
	}
}









//环岛状态判断
void ringL_task(void)
{
	switch(ringR_flag_task)
	{
		case 1:													//1准备进岛

			ring_flag_ing = 200;
			ringR_flag_execute=1;
			distance_ringR += speed_avl;
			
			if(distance_ringR>distance_ringR_1)							//2到达打角点
			{	
				angle_ringR = 0;
				distance_ringR=0;
				ringR_flag_task=2;
				ringR_flag_execute=2;	
				angle_clear();	
			}
			break;
			

		case 2:								
											//1进岛角度积分
		
			if(angle_ringR<-50)						//2打角完成
			{
				ringR_flag_task=3;
				ringR_flag_execute=3;
			}
			break;
			
			
		case 3:
											//1岛内角度积分
		
			if(angle_ringR<-260)					//2即将到达打角点
			{							//335
				ringR_flag_task=4;
				ringR_flag_execute=4;
			}
			break;
			
			
		case 4:
											//1岛内角度积分
			if(angle_ringR<-340)			
			{
				
				ringR_flag_task=5;
				ringR_flag_execute=5;
			}
			break;
		
		case 5:
			
			distance_ringR += speed_avl;
			
			if(distance_ringR>5000)							//2到达打角点
			{	
				angle_clear();
				distance_ringR=0;
				ringR_flag_task=1;
				ringR_flag_execute=1;
				element=0;
				distance_ringR=0;
			}
			
	
		default:
			break;
	
	}

}

//环岛任务执行
void ringL_execute(void)
{
	static float expect_gyro;
	switch(ringR_flag_execute){
		case 1:															//1-直线x进岛			
			speed_target=speed_ringR;					
			gyro_loop(0);
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}
			motor_L(out_L);
	    motor_R(out_R);
			break;
		
    case 2:															//2-打角进岛
			speed_target = speed_ringR;
			gyro_loop(-gyro_ring_in);		
		
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}
			motor_L(out_L);
	    motor_R(out_R);
			break;
		
		case 3:															//3-岛内循迹
			speed_target = speed_ringR;			
			gyro_loop(-gyro_ring_middle);		
		
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}

			motor_L(out_L);
	    motor_R(out_R);
			break;
		
		case 4:															//固定角速度出岛
			speed_target = speed_ringR;			
			gyro_loop(-gyro_ring_out);		
		
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}
			motor_L(out_L);
	    motor_R(out_R);
			break;
		
		case 5:															//直线x出岛
			speed_target=speed_straight;					
			gyro_loop(0);
			if(out_gyro>0)
			{
				loop_speed_LR(speed_target + small_t*out_gyro,speed_target - large_t*out_gyro);
			}
			else
			{
				loop_speed_LR(speed_target + large_t*out_gyro,speed_target - small_t*out_gyro);
			}			motor_L(out_L);
	    motor_R(out_R);
		
		default:break;
	}
}
