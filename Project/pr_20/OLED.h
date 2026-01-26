/*********************************************************************************************************************
 * @file        OLED.h
 * @brief       128x64 OLED 显示屏驱动头文件 (基于软件I2C)
 * @platform    STM32F103C8T6
 * @interface   I2C (软件模拟: PB6=SCL, PB7=SDA)
 * @chip        SSD1306
 *
 * @API分类:
 *  1. 基础控制: 初始化、清屏、开关
 *  2. 兼容API: 行列坐标显示 (原工程风格)
 *  3. 扩展API: 像素坐标显示 + 中文支持
********************************************************************************************************************/

#ifndef __OLED_H
#define __OLED_H

#include "stm32f10x.h"

/*==================================================================================================================*/
/* =============== 常量定义 =============== */
/*==================================================================================================================*/

/** OLED设备地址 (7位地址0x3C << 1) */
#define OLED_ADDRESS    0x78

/** OLED屏幕尺寸 */
#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_PAGES      8       // 64像素 / 8每页 = 8页

/*==================================================================================================================*/
/* =============== 基础控制API =============== */
/*==================================================================================================================*/

/**
 * @brief   初始化OLED显示屏
 * @note    内部调用soft_iic_init()初始化I2C接口 (PB12=SCL, PB13=SDA)
 */
void OLED_Init(void);

/**
 * @brief   清空整个屏幕（填充0x00）
 */
void OLED_Clear(void);

/**
 * @brief   唤醒OLED（退出休眠模式）
 */
void OLED_ON(void);

/**
 * @brief   使OLED进入休眠模式（低功耗）
 */
void OLED_OFF(void);

/*==================================================================================================================*/
/* =============== 兼容原工程API (行列坐标) =============== */
/*==================================================================================================================*/

/**
 * @brief           显示单个字符 (8x16字体)
 * @param   Line    行号 (1~4)
 * @param   Column  列号 (1~16)
 * @param   Char    要显示的字符
 * @note            每行可显示16个字符，共4行
 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);

/**
 * @brief           显示字符串 (8x16字体)
 * @param   Line    起始行号 (1~4)
 * @param   Column  起始列号 (1~16)
 * @param   String  要显示的字符串
 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);

/**
 * @brief           显示无符号十进制数
 * @param   Line    行号 (1~4)
 * @param   Column  列号 (1~16)
 * @param   Number  要显示的数字 (0~4294967295)
 * @param   Length  显示长度 (1~10)
 */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/**
 * @brief           显示有符号十进制数
 * @param   Line    行号 (1~4)
 * @param   Column  列号 (1~16)
 * @param   Number  要显示的数字 (-2147483648~2147483647)
 * @param   Length  数字部分长度 (1~10, 不含符号)
 */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);

/**
 * @brief           显示十六进制数
 * @param   Line    行号 (1~4)
 * @param   Column  列号 (1~16)
 * @param   Number  要显示的数字 (0~0xFFFFFFFF)
 * @param   Length  显示长度 (1~8)
 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/**
 * @brief           显示二进制数
 * @param   Line    行号 (1~4)
 * @param   Column  列号 (1~16)
 * @param   Number  要显示的数字 (0~0xFFFFFFFF)
 * @param   Length  显示长度 (1~16)
 */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/*==================================================================================================================*/
/* =============== 扩展API (像素坐标 + 中文) =============== */
/*==================================================================================================================*/

/**
 * @brief           显示ASCII字符串
 * @param   x       起始列坐标 (0~127)
 * @param   y       起始页坐标 (0~7)
 * @param   ch[]    要显示的字符串
 * @param   TextSize 字体大小: 1=6x8, 2=8x16
 */
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);

/**
 * @brief           显示中文字符 (16x16点阵)
 * @param   x       起始列坐标 (0~127)
 * @param   y       起始页坐标 (0~7)
 * @param   N       codetab.h中的汉字索引
 */
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N);

/**
 * @brief           显示位图图像
 * @param   x0, y0  起始坐标 (x0:0~127, y0:0~7)
 * @param   x1, y1  结束坐标 (像素数: x1:1~128, y1:1~8)
 * @param   BMP[]   位图数据数组
 */
void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[]);

#endif /* __OLED_H */
