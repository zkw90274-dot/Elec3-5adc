/*********************************************************************************************************************
 * @file        OLED.c
 * @brief       128x64 OLED 显示屏驱动实现 (基于软件I2C)
 * @platform    STM32F103C8T6
 * @interface   I2C (软件模拟: PB6=SCL, PB7=SDA)
 * @chip        SSD1306
 *
 * @功能说明:
 *  - 支持两种字体: 6x8 和 8x16 ASCII
 *  - 支持 16x16 中文字符
 *  - 支持位图显示
 *  - 提供行列坐标和像素坐标两套API
 *  - 使用 ZF 风格的 soft_iic API
********************************************************************************************************************/

#include "stm32f10x.h"
#include "OLED.h"
#include "myiic.h"
#include "Delay.h"
#include "codetab.h"

/*==================================================================================================================*/
/* =============== 内部变量 =============== */
/*==================================================================================================================*/

/** OLED的I2C配置对象 */
static soft_iic_info_struct g_oled_iic;

/*==================================================================================================================*/
/* =============== 内部函数 =============== */
/*==================================================================================================================*/

/**
 * @brief       检测I2C设备是否存在
 * @return      1=设备存在, 0=设备不存在
 */
static uint8_t OLED_CheckAddress(uint8_t addr)
{
    uint8_t temp;

    // 尝试写设备地址
    soft_iic_start(&g_oled_iic);
    temp = soft_iic_send_data(&g_oled_iic, (addr << 1) | 0x00);
    soft_iic_stop(&g_oled_iic);

    // 如果收到ACK(返回0)，说明设备存在
    return (temp == 0) ? 1 : 0;
}

/**
 * @brief       自动扫描并设置OLED的I2C地址
 * @note        尝试常见地址: 0x3C 和 0x3D
 */
static void OLED_ScanAddress(void)
{
    if (OLED_CheckAddress(0x3C))
    {
        g_oled_iic.addr = 0x3C;
    }
    else if (OLED_CheckAddress(0x3D))
    {
        g_oled_iic.addr = 0x3D;
    }
    // 如果都失败，保持默认值0x3C
}

/**
 * @brief       写命令到OLED
 * @param   cmd 要写的命令
 * @note        I2C协议: START + DEV_ADDR + 0x00 + CMD + STOP
 */
static void OLED_WriteCommand(uint8_t cmd)
{
    soft_iic_start(&g_oled_iic);
    soft_iic_send_data(&g_oled_iic, (g_oled_iic.addr << 1));  // 设备地址 (写模式)
    soft_iic_send_data(&g_oled_iic, 0x00);     // 控制字节0x00=命令
    soft_iic_send_data(&g_oled_iic, cmd);
    soft_iic_stop(&g_oled_iic);
    Delay_us(10);                              // 命令间延时
}

/**
 * @brief       写数据到OLED
 * @param   dat 要写的数据
 * @note        I2C协议: START + DEV_ADDR + 0x40 + DATA + STOP
 */
static void OLED_WriteData(uint8_t dat)
{
    soft_iic_start(&g_oled_iic);
    soft_iic_send_data(&g_oled_iic, (g_oled_iic.addr << 1));  // 设备地址 (写模式)
    soft_iic_send_data(&g_oled_iic, 0x40);     // 控制字节0x40=数据
    soft_iic_send_data(&g_oled_iic, dat);
    soft_iic_stop(&g_oled_iic);
    Delay_us(10);                              // 数据间延时
}

/**
 * @brief       设置光标位置
 * @param   x 列地址 (0~127)
 * @param   y 页地址 (0~7, 每页8像素)
 */
static void OLED_SetPos(uint8_t x, uint8_t y)
{
    OLED_WriteCommand(0xB0 + y);                      // 设置页地址
    OLED_WriteCommand(((x & 0xF0) >> 4) | 0x10);     // 设置列高4位
    OLED_WriteCommand(x & 0x0F);                      // 设置列低4位 (修复：移除 |0x01)
}

/**
 * @brief       计算X的Y次方
 * @param   X 底数
 * @param   Y 指数
 * @return  计算结果
 */
static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t result = 1;
    while (Y--)
    {
        result *= X;
    }
    return result;
}

/*==================================================================================================================*/
/* =============== 基础控制API =============== */
/*==================================================================================================================*/

/**
 * @brief   初始化OLED显示屏
 * @note    初始化序列参考SSD1306 datasheet
 */
void OLED_Init(void)
{
    Delay_ms(100);                    // 上电延时等待OLED稳定

    // 初始化I2C接口 (使用ZF风格API)
    // 接线: PB12=SCL, PB13=SDA
    soft_iic_init(&g_oled_iic, 0x3C, 20, GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13);  // 增加延时到20us

    // 自动检测OLED的I2C地址 (尝试0x3C和0x3D)
    OLED_ScanAddress();

    // SSD1306初始化命令序列
    OLED_WriteCommand(0xAE);          // 关闭显示
    OLED_WriteCommand(0x20);          // 设置内存地址模式
    OLED_WriteCommand(0x10);          // 页地址模式
    OLED_WriteCommand(0xB0);          // 设置页起始地址
    OLED_WriteCommand(0xC8);          // COM扫描方向: 从下到上
    OLED_WriteCommand(0x00);          // 设置列低地址
    OLED_WriteCommand(0x10);          // 设置列高地址
    OLED_WriteCommand(0x40);          // 设置显示起始行
    OLED_WriteCommand(0x81);          // 设置对比度
    OLED_WriteCommand(0xFF);          // 对比度值: 0~255
    OLED_WriteCommand(0xA1);          // 段重映射: 列127映射到SEG0
    OLED_WriteCommand(0xA6);          // 正常显示
    OLED_WriteCommand(0xA8);          // 设置复用比
    OLED_WriteCommand(0x3F);          // 1/64 duty
    OLED_WriteCommand(0xA4);          // 输出跟随RAM内容
    OLED_WriteCommand(0xD3);          // 设置显示偏移
    OLED_WriteCommand(0x00);          // 无偏移
    OLED_WriteCommand(0xD5);          // 设置显示时钟分频
    OLED_WriteCommand(0xF0);          // 分频比
    OLED_WriteCommand(0xD9);          // 设置预充电周期
    OLED_WriteCommand(0x22);          // 预充电值
    OLED_WriteCommand(0xDA);          // 设置COM引脚配置
    OLED_WriteCommand(0x12);          // 引脚配置
    OLED_WriteCommand(0xDB);          // 设置VCOMH电压
    OLED_WriteCommand(0x20);          // VCOMH电压值
    OLED_WriteCommand(0x8D);          // 设置电荷泵
    OLED_WriteCommand(0x14);          // 使能电荷泵
    OLED_WriteCommand(0xAF);          // 开启显示

    Delay_ms(50);                     // 初始化完成后等待
    OLED_Clear();                     // 清屏
    Delay_ms(50);                     // 清屏后等待

    // 确保显示开启
    OLED_WriteCommand(0xAF);          // 开启显示
}

/**
 * @brief   清空整个屏幕（填充0x00）
 */
void OLED_Clear(void)
{
    uint8_t page, col;

    for (page = 0; page < OLED_PAGES; page++)
    {
        // 直接设置页地址和列地址（与商家驱动一致）
        OLED_WriteCommand(0xB0 + page);  // 页地址
        OLED_WriteCommand(0x00);         // 列低地址
        OLED_WriteCommand(0x10);         // 列高地址

        for (col = 0; col < OLED_WIDTH; col++)
        {
            OLED_WriteData(0x00);
        }
    }
}

/**
 * @brief   唤醒OLED（退出休眠模式）
 */
void OLED_ON(void)
{
    OLED_WriteCommand(0x8D);          // 设置电荷泵
    OLED_WriteCommand(0x14);          // 开启电荷泵
    OLED_WriteCommand(0xAF);          // 显示开启
}

/**
 * @brief   使OLED进入休眠模式（低功耗）
 */
void OLED_OFF(void)
{
    OLED_WriteCommand(0x8D);          // 设置电荷泵
    OLED_WriteCommand(0x10);          // 关闭电荷泵
    OLED_WriteCommand(0xAE);          // 显示关闭
}

/*==================================================================================================================*/
/* =============== 扩展API (像素坐标) =============== */
/*==================================================================================================================*/

/**
 * @brief       显示ASCII字符串 (支持6x8和8x16字体)
 * @param   x       起始列坐标 (0~127)
 * @param   y       起始页坐标 (0~7)
 * @param   ch[]    要显示的字符串
 * @param   size    字体大小: 1=6x8, 2=8x16
 */
void OLED_ShowStr(uint8_t x, uint8_t y, uint8_t ch[], uint8_t size)
{
    uint8_t c, i, j = 0;

    if (size == 1)  // 6x8字体
    {
        while (ch[j] != '\0')
        {
            c = ch[j] - 32;           // 字库偏移（从空格开始）
            if (x > 122)              // 自动换行
            {
                x = 0;
                y++;
            }
            OLED_SetPos(x, y);
            for (i = 0; i < 6; i++)
                OLED_WriteData(F6x8[c][i]);
            x += 6;
            j++;
        }
    }
    else if (size == 2)  // 8x16字体
    {
        while (ch[j] != '\0')
        {
            c = ch[j] - 32;
            if (x > 120)              // 自动换行
            {
                x = 0;
                y += 2;               // 8x16占2页
            }
            // 上半部分
            OLED_SetPos(x, y);
            for (i = 0; i < 8; i++)
                OLED_WriteData(F8X16[c * 16 + i]);
            // 下半部分
            OLED_SetPos(x, y + 1);
            for (i = 0; i < 8; i++)
                OLED_WriteData(F8X16[c * 16 + i + 8]);
            x += 8;
            j++;
        }
    }
}

/**
 * @brief       显示中文字符 (16x16点阵)
 * @param   x   起始列坐标 (0~127)
 * @param   y   起始页坐标 (0~7)
 * @param   N   codetab.h中的汉字索引
 */
void OLED_ShowCN(uint8_t x, uint8_t y, uint8_t N)
{
    uint8_t i;
    uint16_t offset = N * 32;         // 每个汉字32字节

    OLED_SetPos(x, y);
    for (i = 0; i < 16; i++)
        OLED_WriteData(F16x16[offset++]);  // 上半部分

    OLED_SetPos(x, y + 1);
    for (i = 0; i < 16; i++)
        OLED_WriteData(F16x16[offset++]);  // 下半部分
}

/**
 * @brief       显示位图图像
 * @param   x0, y0  起始坐标 (x0:0~127, y0:0~7)
 * @param   x1, y1  结束坐标 (像素数: x1:1~128, y1:1~8)
 * @param   BMP[]   位图数据数组
 */
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t BMP[])
{
    uint16_t i = 0;
    uint8_t x, y;
    uint8_t pages = (y1 % 8) ? (y1 / 8 + 1) : (y1 / 8);

    for (y = y0; y < pages; y++)
    {
        OLED_SetPos(x0, y);
        for (x = x0; x < x1; x++)
        {
            OLED_WriteData(BMP[i++]);
        }
    }
}

/*==================================================================================================================*/
/* =============== 兼容原工程API (行列坐标) =============== */
/*==================================================================================================================*/

/**
 * @brief       显示单个字符 (8x16字体)
 * @param   Line    行号 (1~4)
 * @param   Column  列号 (1~16)
 * @param   Char    要显示的字符
 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    uint16_t offset = (Char - ' ') * 16;
    uint8_t x = (Column - 1) * 8;
    uint8_t y = (Line - 1) * 2;

    // 上半部分
    OLED_SetPos(x, y);
    for (i = 0; i < 8; i++)
        OLED_WriteData(F8X16[offset + i]);

    // 下半部分
    OLED_SetPos(x, y + 1);
    for (i = 0; i < 8; i++)
        OLED_WriteData(F8X16[offset + i + 8]);
}

/**
 * @brief       显示字符串 (8x16字体)
 * @param   Line    起始行号 (1~4)
 * @param   Column  起始列号 (1~16)
 * @param   String  要显示的字符串
 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i = 0;
    while (String[i] != '\0')
    {
        OLED_ShowChar(Line, Column + i, String[i]);
        i++;
    }
}

/**
 * @brief       显示无符号十进制数
 * @param   Line    行号 (1~4)
 * @param   Column  列号 (1~16)
 * @param   Number  要显示的数字 (0~4294967295)
 * @param   Length  显示长度 (1~10)
 */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        uint8_t digit = (Number / OLED_Pow(10, Length - i - 1)) % 10;
        OLED_ShowChar(Line, Column + i, digit + '0');
    }
}

/**
 * @brief       显示有符号十进制数
 * @param   Line    行号 (1~4)
 * @param   Column  列号 (1~16)
 * @param   Number  要显示的数字 (-2147483648~2147483647)
 * @param   Length  数字部分长度 (1~10, 不含符号)
 */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i;
    uint32_t absNum;

    // 显示符号
    if (Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        absNum = Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        absNum = -Number;
    }

    // 显示数字
    for (i = 0; i < Length; i++)
    {
        uint8_t digit = (absNum / OLED_Pow(10, Length - i - 1)) % 10;
        OLED_ShowChar(Line, Column + i + 1, digit + '0');
    }
}

/**
 * @brief       显示十六进制数
 * @param   Line    行号 (1~4)
 * @param   Column  列号 (1~16)
 * @param   Number  要显示的数字 (0~0xFFFFFFFF)
 * @param   Length  显示长度 (1~8)
 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        uint8_t nibble = (Number / OLED_Pow(16, Length - i - 1)) % 16;
        char c = (nibble < 10) ? (nibble + '0') : (nibble - 10 + 'A');
        OLED_ShowChar(Line, Column + i, c);
    }
}

/**
 * @brief       显示二进制数
 * @param   Line    行号 (1~4)
 * @param   Column  列号 (1~16)
 * @param   Number  要显示的数字 (0~0xFFFFFFFF)
 * @param   Length  显示长度 (1~16)
 */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        uint8_t bit = (Number / OLED_Pow(2, Length - i - 1)) % 2;
        OLED_ShowChar(Line, Column + i, bit + '0');
    }
}
