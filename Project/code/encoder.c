//头文件引用
#include "encoder.h"


//变量定义
int16 encoder_data_dir_L = 0;								//左编码器
int16 encoder_data_dir_R = 0;								//右编码器


//函数定义
void Encoder_Init(void)
{
    encoder_dir_init(ENCODER_DIR_R, ENCODER_DIR_DIR_R, ENCODER_DIR_PULSE_R);   	// 初始化编码器模块与引脚 带方向增量编码器模式
    encoder_dir_init(ENCODER_DIR_L, ENCODER_DIR_DIR_L, ENCODER_DIR_PULSE_L);    // 初始化编码器模块与引脚 带方向增量编码器模式

}
