// 头文件包含
#include "key.h"

// 全局变量定义
uint8 key_val, key_old, key_down, key_up;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键初始化
// 参数说明     void
// 返回参数     void
// 使用示例     key_init();
// 备注信息     配置6个按键为上拉输入模式
//-------------------------------------------------------------------------------------------------------------------
void key_init(void)
{
    gpio_init(IO_P32, GPI, 1, GPI_PULL_UP);
    gpio_init(IO_P33, GPI, 1, GPI_PULL_UP);
    gpio_init(IO_P36, GPI, 1, GPI_PULL_UP);

    gpio_init(IO_P50, GPI, 1, GPI_PULL_UP);
    gpio_init(IO_P51, GPI, 1, GPI_PULL_UP);
    gpio_init(IO_P37, GPI, 1, GPI_PULL_UP);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键扫描
// 参数说明     void
// 返回参数     uint8           按键键值 1-6，无按键按下返回0
// 使用示例     uint8 key = Key_Scan();
// 备注信息     检测6个按键的状态
//-------------------------------------------------------------------------------------------------------------------
uint8 Key_Scan(void)
{
    uint8 key_temp = 0;
    if(gpio_get_level(IO_P32) == 0) key_temp = 1;
    if(gpio_get_level(IO_P33) == 0) key_temp = 2;
    if(gpio_get_level(IO_P36) == 0) key_temp = 3;
    if(gpio_get_level(IO_P50) == 0) key_temp = 4;
    if(gpio_get_level(IO_P51) == 0) key_temp = 5;
    if(gpio_get_level(IO_P37) == 0) key_temp = 6;
    return key_temp;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键处理
// 参数说明     void
// 返回参数     void
// 使用示例     Key_Disp();
// 备注信息     检测按键按下和释放事件，更新按键状态
//              key_down: 按键按下标志
//              key_up:   按键释放标志
//-------------------------------------------------------------------------------------------------------------------
void Key_Disp(void)
{
    key_val = Key_Scan();
    key_down = key_val & (key_val ^ key_old);
    key_up = ~key_val & (key_val ^ key_old);
    key_old = key_val;

    if(key_down == 1)
    {
        lcd_cnt++;
    }

    if(key_down == 2)
    {
        lcd_cnt++;
    }

    if(key_down == 3)
    {
        lcd_cnt++;
    }

    if(key_down == 4)
    {
        lcd_cnt++;
    }

    if(key_down == 5)
    {
        lcd_cnt++;
    }

    if(key_down == 6)
    {
        lcd_cnt++;
    }
}
