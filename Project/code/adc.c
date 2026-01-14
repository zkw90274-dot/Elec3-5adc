//-------------------------------------------------------------------------------------------------------------------
// 头文件包含
//-------------------------------------------------------------------------------------------------------------------
#include "adc.h"

//-------------------------------------------------------------------------------------------------------------------
// 全局变量定义
//-------------------------------------------------------------------------------------------------------------------
uint16 adc_val_list[CHANNEL_NUMBER] = {0};                      // ADC原始值数组
uint8 channel_index = 0;                                        // 当前通道索引

// ADC通道枚举数组 - 按从左到右顺序排列
adc_channel_enum channel_list[CHANNEL_NUMBER] =
{
    ADC_L1, ADC_L2, ADC_M3, ADC_R4, ADC_R5
};

// ADC最大最小值校准数组 [偶数索引=最大值, 奇数索引=最小值]
// 需要根据实际传感器校准修改这些值
uint16 max_min_adc[2*CHANNEL_NUMBER] = {
    3532, 0,     // L1: 最大值, 最小值
    3522, 0,     // L2: 最大值, 最小值
    3550, 0,     // M3: 最大值, 最小值
    3516, 0,     // R4: 最大值, 最小值
    3548, 0,     // R5: 最大值, 最小值
};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ADC全部通道初始化
// 参数说明     void
// 返回参数     void
// 使用示例     Adc_All_Init();
// 备注信息     初始化5个ADC通道为12位精度
//-------------------------------------------------------------------------------------------------------------------
void Adc_All_Init(void)
{
    adc_init(ADC_L1, ADC_12BIT);         // L1 左侧传感器1
    adc_init(ADC_L2, ADC_12BIT);         // L2 左侧传感器2
    adc_init(ADC_M3, ADC_12BIT);         // M3 中间传感器
    adc_init(ADC_R4, ADC_12BIT);         // R4 右侧传感器4
    adc_init(ADC_R5, ADC_12BIT);         // R5 右侧传感器5
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ADC值限幅
// 参数说明     adc_val         ADC原始值
// 参数说明     index           通道索引 (0-4)
// 返回参数     uint16          限幅后的ADC值
// 使用示例     uint16 val = limit(adc_val, 0);
// 备注信息     将ADC值限制在校准的最大最小值范围内
//-------------------------------------------------------------------------------------------------------------------
uint16 limit(uint16 adc_val, uint8 index)
{
    if(adc_val > max_min_adc[2*index])
    {
        adc_val = max_min_adc[2*index];
    }
    if(adc_val < max_min_adc[2*index+1])
    {
        adc_val = max_min_adc[2*index+1];
    }
    return adc_val;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ADC测试函数
// 参数说明     void
// 返回参数     void
// 使用示例     Adc_Test();
// 备注信息     打印所有通道的ADC原始值和平均值滤波后的值
//-------------------------------------------------------------------------------------------------------------------
void Adc_Test(void)
{
    // 打印原始转换值
    for(channel_index = 0; channel_index < CHANNEL_NUMBER; channel_index++)
    {
        printf("ADC channel %d convert data is %d.\r\n",
               channel_index + 1,
               adc_convert(channel_list[channel_index]));
    }
    system_delay_ms(1000);

    // 打印平均值滤波后的值
    for(channel_index = 0; channel_index < CHANNEL_NUMBER; channel_index++)
    {
        printf("ADC channel %d mean filter convert data is %d.\r\n",
               channel_index + 1,
               adc_mean_filter_convert(channel_list[channel_index], 10));
    }
    system_delay_ms(500);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取所有ADC值并限幅
// 参数说明     void
// 返回参数     void
// 使用示例     Adc_Getval();
// 备注信息     获取5个通道的ADC值，进行10次平均值滤波并限幅
//              结果保存在 adc_val_list[] 数组中
//-------------------------------------------------------------------------------------------------------------------
void Adc_Getval(void)
{
    for(channel_index = 0; channel_index < CHANNEL_NUMBER; channel_index++)
    {
        // 获取ADC值 -> 10次平均值滤波 -> 限幅
        adc_val_list[channel_index] = limit(
            adc_mean_filter_convert(channel_list[channel_index], 10),
            channel_index
        );
    }
}
