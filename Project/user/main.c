/*********************************************************************************************************************
* STC32G Opensourec Library 即（STC32G 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是STC 开源库的一部分
*
* STC32G 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK FOR C251
* 适用平台          STC32G
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2024-08-01        大W            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"


#define PIT_CH_1                          (TIM1_PIT)                 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_CH_2                          (TIM2_PIT)                 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用

void pit_handler_1(void);
void pit_handler_2(void);

float left_target = 0;										//左轮目标值
float right_target = 0;								 	//右轮目标值
uint8 imu_state = 0;                                          // IMU初始化状态

void main()
{
    clock_init(SYSTEM_CLOCK_30M);
    debug_init();

    Lcd_Init();
    key_init();
    Motor_Init();
    Adc_All_Init();
    Encoder_Init();

    // IMU660RB初始化
    imu_state = imu_init();
    if(imu_state == 0)
    {
        printf("IMU init success!\r\n");
    }
    else
    {
        printf("IMU init failed!\r\n");
    }

	//差比和差系数初始化
//	SDSD_init(&SDSD,1.0,1.0,1.0);

// PID初始化 (Kp, Ki, Kd 参数需要根据实际情况调试)
//    pid_init(&pid_motor_left,1.7, 0.22, 0.2);      	    // 左电机PID参数
//    pid_init(&pid_motor_right, 1.7, 0.1, 0.0);     		// 右电机PID参数
//	pid_init(&pid_SDSD,1.0,0,0);								//SDSD的PID参数

// 设置目标速度 (单位: 编码器计数值每5ms)
//    pid_set_target(&pid_motor_left, left_target);            	// 左电机目标速度
//    pid_set_target(&pid_motor_right, right_target);           // 右电机目标速度
//
//	//设置预期差值
//	pid_set_target(&pid_SDSD,0);

// 此处编写用户代码 例如外设初始化代码等
    tim1_irq_handler = pit_handler_1;					  	//重写tim0中断处理函数
		tim2_irq_handler = pit_handler_2;					  	//重写tim0中断处理函数

    pit_ms_init(PIT_CH_1,10);                		// 初始化 PIT 为周期中断 10ms 周期
		pit_ms_init(PIT_CH_2,50);                		// 初始化 PIT 为周期中断 10ms 周期

    while(1)
    {
        // 此处编写需要循环执行的代码
        // 打印编码器数据
        printf("Encoder: R=%d, L=%d\r\n", encoder_data_dir_R, encoder_data_dir_L);

        // 打印IMU欧拉角数据
        printf("IMU: Roll=%.2f, Pitch=%.2f, Yaw=%.2f\r\n",
               imu.euler.roll, imu.euler.pitch, imu.euler.yaw);

        // 延时避免打印过快
        system_delay_ms(100);
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 的周期中断处理函数 这个函数将在 PIT 对应的定时器中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
void pit_handler_1 (void)
{
    // 获取编码器计数
    encoder_data_dir_R = encoder_get_count(ENCODER_DIR_R);
    encoder_data_dir_L = encoder_get_count(ENCODER_DIR_L);


	motor_pid_control(encoder_data_dir_L, encoder_data_dir_R);
    // 清空编码器计数
    encoder_clear_count(ENCODER_DIR_R);
    encoder_clear_count(ENCODER_DIR_L);


}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 的周期中断处理函数 这个函数将在 PIT 对应的定时器中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
void pit_handler_2 (void)
{
	imu_update();


}
