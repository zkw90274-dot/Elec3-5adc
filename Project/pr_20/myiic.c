/*********************************************************************************************************************
 * @file        myiic.c
 * @brief       软件模拟I2C驱动实现 - 参考ZF API设计风格
 * @platform    STM32F103C8T6
 *
 * @设计理念:
 *  1. 单一配置结构体：所有信息封装在 soft_iic_info_struct
 *  2. 统一命名规则：write_8bit / read_8bit / write_8bit_register / read_8bit_register
 *  3. 简洁易用：所有操作函数只接受一个结构体指针参数
 *  4. 完全解耦：GPIO初始化在soft_iic_init中完成
********************************************************************************************************************/

#include "myiic.h"
#include "Delay.h"

/*==================================================================================================================*/
/* =============== 内部函数 =============== */
/*==================================================================================================================*/

/**
 * @brief       延时函数
 */
static void iic_delay(uint32_t delay)
{
    Delay_us(delay);
}

/**
 * @brief       设置GPIO为输入模式 (SDA读取)
 */
static inline void sda_input(GPIO_TypeDef *port, gpio_pin_enum sda)
{
    uint16_t pin = (uint16_t)sda;
    uint32_t tmp, mask;

    if (pin <= 7)
    {
        tmp = port->CRL;
        mask = 0xF << (pin * 4);
        tmp = (tmp & ~mask) | (0x8 << (pin * 4));
        port->CRL = tmp;
    }
    else
    {
        tmp = port->CRH;
        mask = 0xF << ((pin - 8) * 4);
        tmp = (tmp & ~mask) | (0x8 << ((pin - 8) * 4));
        port->CRH = tmp;
    }
}

/**
 * @brief       设置GPIO为输出模式 (SDA写入)
 */
static inline void sda_output(GPIO_TypeDef *port, gpio_pin_enum sda)
{
    uint16_t pin = (uint16_t)sda;
    uint32_t tmp, mask;

    if (pin <= 7)
    {
        tmp = port->CRL;
        mask = 0xF << (pin * 4);
        tmp = (tmp & ~mask) | (0x3 << (pin * 4));
        port->CRL = tmp;
    }
    else
    {
        tmp = port->CRH;
        mask = 0xF << ((pin - 8) * 4);
        tmp = (tmp & ~mask) | (0x3 << ((pin - 8) * 4));
        port->CRH = tmp;
    }
}

/**
 * @brief       设置GPIO电平
 */
static inline void gpio_set_level(GPIO_TypeDef *port, gpio_pin_enum pin, uint8_t level)
{
    if (level)
        port->BSRR = (uint16_t)pin;
    else
        port->BRR = (uint16_t)pin;
}

/**
 * @brief       读取GPIO电平
 */
static inline uint8_t gpio_get_level(GPIO_TypeDef *port, gpio_pin_enum pin)
{
    return (port->IDR & (uint16_t)pin) ? 1 : 0;
}

/*==================================================================================================================*/
/* =============== 内部协议API实现 =============== */
/*==================================================================================================================*/

/**
 * @brief       产生I2C起始信号
 */
void soft_iic_start(soft_iic_info_struct *soft_iic_obj)
{
    sda_output(soft_iic_obj->sda_port, soft_iic_obj->sda_pin);
    gpio_set_level(soft_iic_obj->scl_port, soft_iic_obj->scl_pin, 1);     // SCL高
    gpio_set_level(soft_iic_obj->sda_port, soft_iic_obj->sda_pin, 1);     // SDA高
    iic_delay(soft_iic_obj->delay);

    gpio_set_level(soft_iic_obj->sda_port, soft_iic_obj->sda_pin, 0);     // SDA下降沿 → START
    iic_delay(soft_iic_obj->delay);
    gpio_set_level(soft_iic_obj->scl_port, soft_iic_obj->scl_pin, 0);     // SCL低
}

/**
 * @brief       产生I2C停止信号
 */
void soft_iic_stop(soft_iic_info_struct *soft_iic_obj)
{
    sda_output(soft_iic_obj->sda_port, soft_iic_obj->sda_pin);
    gpio_set_level(soft_iic_obj->sda_port, soft_iic_obj->sda_pin, 0);     // SDA低
    gpio_set_level(soft_iic_obj->scl_port, soft_iic_obj->scl_pin, 0);     // SCL低
    iic_delay(soft_iic_obj->delay);

    gpio_set_level(soft_iic_obj->scl_port, soft_iic_obj->scl_pin, 1);     // SCL高
    iic_delay(soft_iic_obj->delay);
    gpio_set_level(soft_iic_obj->sda_port, soft_iic_obj->sda_pin, 1);     // SDA上升沿 → STOP
    iic_delay(soft_iic_obj->delay);
}

/**
 * @brief       发送应答信号
 * @param   ack     0=ACK, 1=NACK
 */
void soft_iic_send_ack(soft_iic_info_struct *soft_iic_obj, uint8_t ack)
{
    gpio_set_level(soft_iic_obj->scl_port, soft_iic_obj->scl_pin, 0);     // SCL低

    if (ack)
        gpio_set_level(soft_iic_obj->sda_port, soft_iic_obj->sda_pin, 1); // NACK
    else
        gpio_set_level(soft_iic_obj->sda_port, soft_iic_obj->sda_pin, 0); // ACK

    iic_delay(10);
    gpio_set_level(soft_iic_obj->scl_port, soft_iic_obj->scl_pin, 1);     // SCL高(发送ACK)
    iic_delay(10);
    gpio_set_level(soft_iic_obj->scl_port, soft_iic_obj->scl_pin, 0);     // SCL低
    gpio_set_level(soft_iic_obj->sda_port, soft_iic_obj->sda_pin, 1);     // 释放SDA
}

/**
 * @brief       等待从机应答
 * @return  0=ACK, 1=NACK/超时
 */
uint8_t soft_iic_wait_ack(soft_iic_info_struct *soft_iic_obj)
{
    uint16_t timeout = 0;

    gpio_set_level(soft_iic_obj->scl_port, soft_iic_obj->scl_pin, 0);     // SCL低
    sda_input(soft_iic_obj->sda_port, soft_iic_obj->sda_pin);             // SDA输入
    gpio_set_level(soft_iic_obj->sda_port, soft_iic_obj->sda_pin, 1);     // 释放SDA
    iic_delay(soft_iic_obj->delay);

    gpio_set_level(soft_iic_obj->scl_port, soft_iic_obj->scl_pin, 1);     // SCL高
    iic_delay(soft_iic_obj->delay);

    // 等待SDA被拉低(ACK)
    while (gpio_get_level(soft_iic_obj->sda_port, soft_iic_obj->sda_pin))
    {
        if (++timeout > 250)     // 超时保护
        {
            sda_output(soft_iic_obj->sda_port, soft_iic_obj->sda_pin);
            soft_iic_stop(soft_iic_obj);
            return 1;           // 超时返回NACK
        }
        iic_delay(1);
    }

    gpio_set_level(soft_iic_obj->scl_port, soft_iic_obj->scl_pin, 0);      // SCL低
    sda_output(soft_iic_obj->sda_port, soft_iic_obj->sda_pin);             // SDA输出
    iic_delay(soft_iic_obj->delay);

    return 0;                                        // 收到ACK
}

/**
 * @brief       发送一个字节
 * @return  0=ACK, 1=NACK
 */
uint8_t soft_iic_send_data(soft_iic_info_struct *soft_iic_obj, uint8_t dat)
{
    uint8_t mask = 0x80;

    sda_output(soft_iic_obj->sda_port, soft_iic_obj->sda_pin);
    while (mask)
    {
        gpio_set_level(soft_iic_obj->scl_port, soft_iic_obj->scl_pin, 0);  // SCL低

        if (dat & mask)
            gpio_set_level(soft_iic_obj->sda_port, soft_iic_obj->sda_pin, 1);
        else
            gpio_set_level(soft_iic_obj->sda_port, soft_iic_obj->sda_pin, 0);

        iic_delay(soft_iic_obj->delay);
        gpio_set_level(soft_iic_obj->scl_port, soft_iic_obj->scl_pin, 1);  // SCL高(数据被采样)
        iic_delay(soft_iic_obj->delay);

        mask >>= 1;
    }

    return soft_iic_wait_ack(soft_iic_obj);
}

/**
 * @brief       读取一个字节
 * @param   ack     0=发送ACK, 1=发送NACK
 * @return  读取的数据
 */
uint8_t soft_iic_read_data(soft_iic_info_struct *soft_iic_obj, uint8_t ack)
{
    uint8_t dat = 0;
    uint8_t i;

    gpio_set_level(soft_iic_obj->scl_port, soft_iic_obj->scl_pin, 0);     // SCL低
    sda_input(soft_iic_obj->sda_port, soft_iic_obj->sda_pin);             // SDA输入
    gpio_set_level(soft_iic_obj->sda_port, soft_iic_obj->sda_pin, 1);     // 释放SDA
    iic_delay(soft_iic_obj->delay);

    for (i = 0; i < 8; i++)
    {
        gpio_set_level(soft_iic_obj->scl_port, soft_iic_obj->scl_pin, 0);  // SCL低
        iic_delay(soft_iic_obj->delay);
        gpio_set_level(soft_iic_obj->scl_port, soft_iic_obj->scl_pin, 1);  // SCL高(读取数据)
        iic_delay(soft_iic_obj->delay);
        dat = (dat << 1) | gpio_get_level(soft_iic_obj->sda_port, soft_iic_obj->sda_pin);
    }

    gpio_set_level(soft_iic_obj->scl_port, soft_iic_obj->scl_pin, 0);      // SCL低
    sda_output(soft_iic_obj->sda_port, soft_iic_obj->sda_pin);             // SDA输出
    iic_delay(soft_iic_obj->delay);
    soft_iic_send_ack(soft_iic_obj, ack);          // 发送ACK/NACK

    return dat;
}

/*==================================================================================================================*/
/* =============== 基础操作API实现 =============== */
/*==================================================================================================================*/

/**
 * @brief       写8位数据到I2C设备
 */
void soft_iic_write_8bit(soft_iic_info_struct *soft_iic_obj, const uint8_t dat)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);  // 写模式
    soft_iic_send_data(soft_iic_obj, dat);
    soft_iic_stop(soft_iic_obj);
}

/**
 * @brief       写8位数组到I2C设备
 */
void soft_iic_write_8bit_array(soft_iic_info_struct *soft_iic_obj, const uint8_t *dat, uint32_t len)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);  // 写模式
    while (len--)
        soft_iic_send_data(soft_iic_obj, *dat++);
    soft_iic_stop(soft_iic_obj);
}

/**
 * @brief       从I2C设备读取8位数据
 */
uint8_t soft_iic_read_8bit(soft_iic_info_struct *soft_iic_obj)
{
    uint8_t dat;
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, (soft_iic_obj->addr << 1) | 0x01);  // 读模式
    dat = soft_iic_read_data(soft_iic_obj, 1);                               // NACK
    soft_iic_stop(soft_iic_obj);
    return dat;
}

/**
 * @brief       从I2C设备读取8位数组
 */
void soft_iic_read_8bit_array(soft_iic_info_struct *soft_iic_obj, uint8_t *dat, uint32_t len)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, (soft_iic_obj->addr << 1) | 0x01);  // 读模式
    while (len--)
        *dat++ = soft_iic_read_data(soft_iic_obj, (len == 0) ? 1 : 0);  // 最后字节NACK
    soft_iic_stop(soft_iic_obj);
}

/*==================================================================================================================*/
/* =============== 寄存器操作API实现 =============== */
/*==================================================================================================================*/

/**
 * @brief       写8位寄存器
 */
void soft_iic_write_8bit_register(soft_iic_info_struct *soft_iic_obj, const uint8_t register_name, const uint8_t dat)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);  // 写模式
    soft_iic_send_data(soft_iic_obj, register_name);            // 寄存器地址
    soft_iic_send_data(soft_iic_obj, dat);
    soft_iic_stop(soft_iic_obj);
}

/**
 * @brief       写8位寄存器数组
 */
void soft_iic_write_8bit_registers(soft_iic_info_struct *soft_iic_obj, const uint8_t register_name, const uint8_t *dat, uint32_t len)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);  // 写模式
    soft_iic_send_data(soft_iic_obj, register_name);            // 寄存器地址
    while (len--)
        soft_iic_send_data(soft_iic_obj, *dat++);
    soft_iic_stop(soft_iic_obj);
}

/**
 * @brief       读8位寄存器
 */
uint8_t soft_iic_read_8bit_register(soft_iic_info_struct *soft_iic_obj, const uint8_t register_name)
{
    uint8_t dat;
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);            // 写模式
    soft_iic_send_data(soft_iic_obj, register_name);                       // 寄存器地址
    soft_iic_start(soft_iic_obj);                                           // 重复START
    soft_iic_send_data(soft_iic_obj, (soft_iic_obj->addr << 1) | 0x01);    // 读模式
    dat = soft_iic_read_data(soft_iic_obj, 1);                              // NACK
    soft_iic_stop(soft_iic_obj);
    return dat;
}

/**
 * @brief       读8位寄存器数组
 */
void soft_iic_read_8bit_registers(soft_iic_info_struct *soft_iic_obj, const uint8_t register_name, uint8_t *dat, uint32_t len)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);            // 写模式
    soft_iic_send_data(soft_iic_obj, register_name);                       // 寄存器地址
    soft_iic_start(soft_iic_obj);                                           // 重复START
    soft_iic_send_data(soft_iic_obj, (soft_iic_obj->addr << 1) | 0x01);    // 读模式
    while (len--)
        *dat++ = soft_iic_read_data(soft_iic_obj, (len == 0) ? 1 : 0);
    soft_iic_stop(soft_iic_obj);
}

/*==================================================================================================================*/
/* =============== 高级传输API实现 =============== */
/*==================================================================================================================*/

/**
 * @brief       I2C传输8位数组 (先写后读)
 */
void soft_iic_transfer_8bit_array(soft_iic_info_struct *soft_iic_obj, const uint8_t *write_data, uint32_t write_len,
                                   uint8_t *read_data, uint32_t read_len)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);
    while (write_len--)
    {
        soft_iic_send_data(soft_iic_obj, *write_data++);
    }

    if (read_len)
    {
        soft_iic_start(soft_iic_obj);
        soft_iic_send_data(soft_iic_obj, (soft_iic_obj->addr << 1) | 0x01);
        while (read_len--)
        {
            *read_data++ = soft_iic_read_data(soft_iic_obj, (read_len == 0) ? 1 : 0);
        }
    }
    soft_iic_stop(soft_iic_obj);
}

/*==================================================================================================================*/
/* =============== 初始化API实现 =============== */
/*==================================================================================================================*/

/**
 * @brief       初始化软件I2C
 * @note        支持任意GPIO端口和引脚组合
 */
void soft_iic_init(soft_iic_info_struct *soft_iic_obj, uint8_t addr, uint32_t delay,
                   GPIO_TypeDef *scl_port, gpio_pin_enum scl_pin,
                   GPIO_TypeDef *sda_port, gpio_pin_enum sda_pin)
{
    GPIO_InitTypeDef gpio_cfg = {0};

    if (soft_iic_obj == NULL || scl_pin == sda_pin) return;

    // 保存配置到结构体
    soft_iic_obj->scl_port = scl_port;
    soft_iic_obj->scl_pin = scl_pin;
    soft_iic_obj->sda_port = sda_port;
    soft_iic_obj->sda_pin = sda_pin;
    soft_iic_obj->addr = addr;
    soft_iic_obj->delay = delay;

    // 根据端口使能对应的RCC时钟
    if (scl_port == sda_port)
    {
        // 同一端口，只需使能一次
        if (scl_port == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        else if (scl_port == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        else if (scl_port == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        else if (scl_port == GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

        // 配置GPIO为开漏输出模式
        gpio_cfg.GPIO_Pin = (uint16_t)scl_pin | (uint16_t)sda_pin;
        gpio_cfg.GPIO_Mode = GPIO_Mode_Out_OD;
        gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(scl_port, &gpio_cfg);
    }
    else
    {
        // 不同端口，分别使能时钟并初始化
        if (scl_port == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        else if (scl_port == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        else if (scl_port == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        else if (scl_port == GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

        if (sda_port == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        else if (sda_port == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        else if (sda_port == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        else if (sda_port == GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

        // 分别初始化SCL和SDA
        gpio_cfg.GPIO_Pin = (uint16_t)scl_pin;
        gpio_cfg.GPIO_Mode = GPIO_Mode_Out_OD;
        gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(scl_port, &gpio_cfg);

        gpio_cfg.GPIO_Pin = (uint16_t)sda_pin;
        GPIO_Init(sda_port, &gpio_cfg);
    }

    // 初始状态为高电平
    gpio_set_level(scl_port, scl_pin, 1);
    gpio_set_level(sda_port, sda_pin, 1);
    Delay_us(10);
}
