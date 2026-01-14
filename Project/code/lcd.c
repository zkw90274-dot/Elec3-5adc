//头文件引用
#include "lcd.h"


//变量定义

uint8 lcd_cnt = 0;

//函数定义

void Lcd_Init(void)
{
    tft180_init();
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_clear(RGB565_BLACK);

}

void Lcd_Disp(void)
{

    tft180_show_string(0, 0, "lcd cnt");
}