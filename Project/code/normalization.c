#include "normalization.h"

// 全局变量定义
uint16 adc_normalized_list[CHANNEL_NUMBER] = {0};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ADC值归一化处理
// 参数说明     void
// 返回参数     void
// 使用示例     Normalization();
// 备注信息     将adc_val_list[]中的ADC原始值归一化到0-50范围
//              归一化公式: result = 50 * (adc_val - min) / (max - min)
//-------------------------------------------------------------------------------------------------------------------
void Normalization(void)
{
    float temp;
    float range;

    for(channel_index = 0; channel_index < CHANNEL_NUMBER; channel_index++)
    {
        range = max_min_adc[2*channel_index] - max_min_adc[2*channel_index+1];
        temp = (float)(adc_val_list[channel_index] - max_min_adc[2*channel_index+1]);
        temp = 50.0f * temp / range;
        adc_normalized_list[channel_index] = (uint16)temp;
    }
}
