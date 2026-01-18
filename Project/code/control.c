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
//              适配5路传感器布局: [0][1][2][3][4]
//              使用全部5个传感器计算加权偏差
//              负数=偏左, 正数=偏右, 0=居中
//              计算公式: result = 100*(右-左) / (右+左+|中|+1)
//-------------------------------------------------------------------------------------------------------------------
float SDSD_calculate(SDSD_param_t *SDSD)
{
    // 5路传感器布局: [0]  [1]  [2]  [3]  [4]
    //                最左  左   中   右  最右
    //                权重: 1   2   3   2   1

    // 差值计算: 右侧传感器和 - 左侧传感器和
    // 系数A: 主要传感器(1,3)权重
    // 系数B: 次要传感器(0,4)权重
    sub = (SDSD->parallel_err_A * (adc_normalized_list[3] + adc_normalized_list[4] -
                                   adc_normalized_list[1] - adc_normalized_list[0]) +
           SDSD->vertical_err_B * (adc_normalized_list[3] - adc_normalized_list[1])) * 100;

    // 和值计算: 所有传感器激活程度 + 1（防止除零）
    // 系数C: 中心传感器(2)的归一化权重
    sum = SDSD->parallel_err_A * (adc_normalized_list[0] + adc_normalized_list[1] +
                                  adc_normalized_list[3] + adc_normalized_list[4]) +
          SDSD->vertival_err_C * adc_normalized_list[2] + 1;

    res = sub / sum;

    // 限幅
    if(res > SDSD_MAX)
        res = SDSD_MAX;
    else if(res < SDSD_MIN)
        res = SDSD_MIN;

    return res;
}
