#ifndef _ENCODER_H_
#define _ENCODER_H_

//头文件引用
#include "zf_common_headfile.h"
//变量声明

extern	int16 encoder_data_dir_L;
extern  int16 encoder_data_dir_R;

//左编码器
#define ENCODER_DIR_R                 	(TIM0_ENCOEDER)                         // 正交编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER1
#define ENCODER_DIR_DIR_R              	(IO_P35)            				 	// DIR 对应的引脚
#define ENCODER_DIR_PULSE_R            	(TIM0_ENCOEDER_P34)            			// PULSE 对应的引脚
//右编码器
#define ENCODER_DIR_L                 	(TIM3_ENCOEDER)                         // 带方向编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER2
#define ENCODER_DIR_DIR_L           	(IO_P53)             					// DIR 对应的引脚
#define ENCODER_DIR_PULSE_L       		(TIM3_ENCOEDER_P04)            			// PULSE 对应的引脚
//函数声明

void Encoder_Init(void);



#endif