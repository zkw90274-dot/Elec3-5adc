//-------------------------------------------------------------------------------------------------------------------
// 头文件包含
//-------------------------------------------------------------------------------------------------------------------
#include "encoder.h"

//-------------------------------------------------------------------------------------------------------------------
// 全局变量定义
//-------------------------------------------------------------------------------------------------------------------
int16 encoder_data_dir_L = 0;                                          // 左编码器计数值
int16 encoder_data_dir_R = 0;                                          // 右编码器计数值

// 一阶IIR低通滤波器状态变量
static float last_encoder_L = 0.0f;                                    // 左编码器上次滤波值
static float last_encoder_R = 0.0f;                                    // 右编码器上次滤波值

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器初始化
// 参数说明     void
// 返回参数     void
// 使用示例     Encoder_Init();
// 备注信息     初始化左右编码器的硬件接口和滤波器状态
//-------------------------------------------------------------------------------------------------------------------
void Encoder_Init(void)
{
    encoder_dir_init(ENCODER_DIR_R, ENCODER_DIR_DIR_R, ENCODER_DIR_PULSE_R);
    encoder_dir_init(ENCODER_DIR_L, ENCODER_DIR_DIR_L, ENCODER_DIR_PULSE_L);
    last_encoder_L = 0.0f;
    last_encoder_R = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     一阶IIR低通滤波器
// 参数说明     new_val         新采样值
// 参数说明     last_val        上次滤波值
// 参数说明     alpha           滤波系数 (0.0-1.0), 越小滤波越强
// 返回参数     float           滤波后的值
// 使用示例     filtered = iir_lowpass(new_value, last_value, 0.2f);
// 备注信息     一阶IIR低通滤波器公式: y[n] = alpha * x[n] + (1-alpha) * y[n-1]
//              alpha = 1.0  -> 无滤波, alpha = 0.1  -> 强滤波
//              推荐值: 0.1-0.3
//-------------------------------------------------------------------------------------------------------------------
float iir_lowpass(float new_val, float last_val, float alpha)
{
    return alpha * new_val + (1.0f - alpha) * last_val;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取滤波后的编码器值
// 参数说明     void
// 返回参数     void
// 使用示例     Encoder_Get_Filtered();
// 备注信息     获取编码器原始值并经过一阶IIR低通滤波
//              滤波系数 alpha = 0.2 (可在函数中调整)
//              在中断中调用，自动完成采集、滤波、清零
//-------------------------------------------------------------------------------------------------------------------
void Encoder_Get_Filtered(void)
{
    float temp_L, temp_R;
    float alpha = 0.2f;

    temp_L = (float)encoder_get_count(ENCODER_DIR_L);
    temp_R = (float)encoder_get_count(ENCODER_DIR_R);

    last_encoder_L = iir_lowpass(temp_L, last_encoder_L, alpha);
    last_encoder_R = iir_lowpass(temp_R, last_encoder_R, alpha);

    encoder_data_dir_L = (int16)last_encoder_L;
    encoder_data_dir_R = (int16)last_encoder_R;

    encoder_clear_count(ENCODER_DIR_L);
    encoder_clear_count(ENCODER_DIR_R);
}
