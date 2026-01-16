#ifndef __ENCODER_H
#define __ENCODER_H


#include "bsp_system.h"


#define DIL P13
#define DIR P35

#define MAX_WINDOW_SIZE 3
// 滑动平均滤波器结构体
typedef struct {
    float buffer[MAX_WINDOW_SIZE]; // 静态缓冲区
    float sum;                     // 当前窗口数值的和
    int index;                     // 当前存储数值的位置
    int count;                     // 当前已填充的数值个数
    int window_size;               // 滑动窗口大小
} SlidingAverageFilter;

extern SlidingAverageFilter filter_Left;
extern SlidingAverageFilter filter_Right;

extern float speed_L,speed_R,speed_avl;
extern float speed_L_next,speed_R_next,speed_avl_next;

extern int32 distance_text;





void encoder_init(void);
void encoder_update(void);
void initSlidingAverage(SlidingAverageFilter* filter, int N);
void slidingAverage(float now_speed, float* avg_speed, SlidingAverageFilter* filter);

#endif