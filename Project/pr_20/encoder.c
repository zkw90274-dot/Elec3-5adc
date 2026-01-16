#include "zf_common_headfile.h"
#include "encoder.h"
#include "motor.h"


float speed_L=0,speed_R=0,speed_avl=0;
float speed_L_next=0,speed_R_next=0,speed_avl_next;
int16 lastspeed_L=0,lastspeed_R=0;
int32 distance_text=0;

SlidingAverageFilter filter_Left;
SlidingAverageFilter filter_Right;

void encoder_init(void)
{
	encoder_dir_init(TIM0_ENCOEDER,IO_P35,TIM0_ENCOEDER_P34);
	encoder_dir_init(TIM3_ENCOEDER,IO_P13,TIM3_ENCOEDER_P04);

	initSlidingAverage(&filter_Left,6);  // 初始化滤波器
	initSlidingAverage(&filter_Right,6);
	
}

void encoder_update(void)
{
	//0-获取速度&清零
	speed_R = (0.85f*encoder_get_count(TIM0_ENCOEDER));
	speed_L = (0.85f*encoder_get_count(TIM3_ENCOEDER));
	
	encoder_clear_count(TIM3_ENCOEDER);
	encoder_clear_count(TIM0_ENCOEDER);
	//1-一阶低通滤波
	speed_L=(0.9*speed_L + 0.1*lastspeed_L);
	speed_R=(0.9*speed_R + 0.1*lastspeed_R);
	lastspeed_L = speed_L;
	lastspeed_R = speed_R;
	 
	speed_avl = (speed_L + speed_R)/2;
	
	distance_text += (int32)speed_avl;
}

//void encoder_update(void)
//{
//	//0-获取速度&清零
//	speed_R_next = (float)-encoder_get_count(TIM0_ENCOEDER);
//	speed_L_next = (float)-encoder_get_count(TIM3_ENCOEDER);
//	
//	encoder_clear_count(TIM3_ENCOEDER);
//	encoder_clear_count(TIM0_ENCOEDER);
//	//1-滑动平均滤波
//	slidingAverage(speed_L_next,&speed_L, &filter_Left);
//	slidingAverage(speed_R_next,&speed_R, &filter_Right);

//	//2-实际速度及其距离
//	speed_avl_next = (speed_R_next + speed_L_next)/2;
//	speed_avl = (speed_L + speed_R)/2;
//	
//	
//	distance_text += (int32)speed_avl;
//}

void initSlidingAverage(SlidingAverageFilter* filter, int N) {
	 uint8 Win_i=0;
    if (N > MAX_WINDOW_SIZE) {
        N = MAX_WINDOW_SIZE; // 限制窗口大小不能超过最大值
    }

    for (Win_i = 0; Win_i < MAX_WINDOW_SIZE; Win_i++) {
        filter->buffer[Win_i] = 0.0f;
    }
    filter->sum = 0.0f;
    filter->index = 0;
    filter->count = 0;
    filter->window_size = N;
}
	
void slidingAverage(float now_speed, float* avg_speed, SlidingAverageFilter* filter) {
    // 从和中减去即将被替换的旧值
    filter->sum -= filter->buffer[filter->index];

    // 将新的值插入缓冲区
    filter->buffer[filter->index] = now_speed;

    // 更新和
    filter->sum += now_speed;

    // 更新索引，使其循环回到起始位置
    filter->index = (filter->index + 1) % filter->window_size;

    // 如果窗口没有满，增加计数
    if (filter->count < filter->window_size) {
        filter->count++;
    }

    // 计算并返回当前窗口的平均值
    *avg_speed = filter->sum / filter->count;
}
