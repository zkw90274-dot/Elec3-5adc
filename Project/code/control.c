#include "control.h"

// 全局变量定义
SDSD_param_t SDSD;
int32 sub;    // 差值
int32 sum;    // 和值
float res;    // 结果值

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     差比和差参数初始化
// 参数说明     SDSD            差比和差结构体指针
// 参数说明     A               水平系数(系数A)
// 参数说明     B               垂直系数(系数B)
// 参数说明     C               垂直系数(系数C)
// 返回参数     void
// 使用示例     SDSD_init(&SDSD, 1.0, 1.0, 1.0);
//-------------------------------------------------------------------------------------------------------------------
void SDSD_init(SDSD_param_t *SDSD, float A, float B, float C)
{
    SDSD->parallel_err_A = A;
    SDSD->vertical_err_B = B;
    SDSD->vertival_err_C = C;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     差比和差计算
// 参数说明     SDSD            差比和差结构体指针
// 返回参数     float           差比和差计算结果
// 使用示例     float result = SDSD_calculate(&SDSD);
// 备注信息     计算差比和差值用于循迹控制
//-------------------------------------------------------------------------------------------------------------------
float SDSD_calculate(SDSD_param_t *SDSD)
{
    sub = (SDSD->parallel_err_A * (adc_normalized_list[1] - adc_normalized_list[7]) +
           SDSD->vertical_err_B * (adc_normalized_list[2] - adc_normalized_list[6])) * 100;

    sum = SDSD->parallel_err_A * (adc_normalized_list[1] - adc_normalized_list[7]) +
          SDSD->vertival_err_C * abs(adc_normalized_list[2] - adc_normalized_list[6]) + 1;

    res = sub / sum;

    if(res > SDSD_MAX)
        res = SDSD_MAX;
    else if(res < SDSD_MIN)
        res = SDSD_MIN;

    return res;
}
