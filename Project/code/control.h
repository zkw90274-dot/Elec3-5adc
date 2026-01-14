#ifndef _CONTROL_H_
#define _CONTROL_H_

//头文件包含
#include "zf_common_headfile.h"

//宏定义
#define SDSD_MAX  (80)        //最大值
#define SDSD_MIN  (-80)       //最小值

//差比和差结构体
typedef struct
{
    float parallel_err_A;    //水平系数(系数A)
    float vertical_err_B;    //垂直系数(系数B)
    float vertival_err_C;    //垂直系数(系数C)

} SDSD_param_t;              //差比和差(Substract Divide Sum Difference)结构体

//外部变量声明
extern SDSD_param_t SDSD;

//函数声明
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     差比和差参数初始化
// 参数说明     SDSD            差比和差结构体指针
// 参数说明     A               水平系数(系数A)
// 参数说明     B               垂直系数(系数B)
// 参数说明     C               垂直系数(系数C)
// 返回参数     void
// 使用示例     SDSD_init(&SDSD, 1.0, 1.0, 1.0);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void SDSD_init(SDSD_param_t *SDSD, float A, float B, float C);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     差比和差计算
// 参数说明     SDSD            差比和差结构体指针
// 返回参数     float           差比和差计算结果
// 使用示例     float result = SDSD_calculate(&SDSD);
// 备注信息     计算差比和差值用于循迹控制
//-------------------------------------------------------------------------------------------------------------------
float SDSD_calculate(SDSD_param_t *SDSD);

// 兼容旧版本的宏定义
#define Result(SDSD)      SDSD_calculate(SDSD)

#endif
