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
//              改进: 添加除零保护、超界限幅
//-------------------------------------------------------------------------------------------------------------------
void Normalization(void)
{
    float temp;
    float range;
    uint8 i;

    for(i = 0; i < CHANNEL_NUMBER; i++)
    {
        // 计算范围，防止除零
        range = (float)(max_min_adc[2*i] - max_min_adc[2*i+1]);
        if(range < 1.0f)
            range = 1.0f;

        // 线性归一化到 0-50
        temp = (float)(adc_val_list[i] - max_min_adc[2*i+1]);
        temp = 50.0f * temp / range;

        // 限幅到有效范围
        if(temp < 0.0f)
            temp = 0.0f;
        else if(temp > 50.0f)
            temp = 50.0f;

        adc_normalized_list[i] = (uint16)temp;
    }
}
