/*********************************************************************************************************************
 * @file        myiic.h
 * @brief       软件模拟I2C驱动 - 参考ZF API设计风格
 * @platform    STM32F103C8T6
 *
 * @API设计理念:
 *  1. 单一配置结构体：所有信息封装在 soft_iic_info_struct
 *  2. 统一命名规则：write_8bit / read_8bit / write_8bit_register / read_8bit_register
 *  3. 简洁易用：所有操作函数只接受一个结构体指针参数
 *  4. 完全解耦：GPIO初始化在soft_iic_init中完成
 *
 * @API层次:
 *  - 内部协议层：start/stop/send_data/read_data (用户无需调用)
 *  - 基础操作层：write_8bit/read_8bit 及数组版本
 *  - 寄存器操作层：write_8bit_register/read_8bit_register 及数组版本
 *  - 高级传输层：transfer_8bit_array (先写后读)
********************************************************************************************************************/

#ifndef __MYIIC_H
#define __MYIIC_H

#include <stddef.h>
#include "stm32f10x.h"

/*==================================================================================================================*/
/* =============== 类型定义 =============== */
/*==================================================================================================================*/

/** GPIO引脚枚举 */
typedef enum
{
    GPIO_PIN_0  = 0x0001,  GPIO_PIN_1  = 0x0002,  GPIO_PIN_2  = 0x0004,  GPIO_PIN_3  = 0x0008,
    GPIO_PIN_4  = 0x0010,  GPIO_PIN_5  = 0x0020,  GPIO_PIN_6  = 0x0040,  GPIO_PIN_7  = 0x0080,
    GPIO_PIN_8  = 0x0100,  GPIO_PIN_9  = 0x0200,  GPIO_PIN_10 = 0x0400, GPIO_PIN_11 = 0x0800,
    GPIO_PIN_12 = 0x1000,  GPIO_PIN_13 = 0x2000, GPIO_PIN_14 = 0x4000, GPIO_PIN_15 = 0x8000,
} gpio_pin_enum;

/**
 * @brief   软件I2C配置结构体 (包含所有配置信息)
 */
typedef struct
{
    GPIO_TypeDef *scl_port;    // SCL端口 (GPIOA/GPIOB/GPIOC...)
    gpio_pin_enum  scl_pin;    // SCL引脚
    GPIO_TypeDef *sda_port;    // SDA端口
    gpio_pin_enum  sda_pin;    // SDA引脚
    uint8_t  addr;             // I2C设备地址 (7位地址模式)
    uint32_t delay;            // 模拟I2C延时时钟 (控制I2C速度)
} soft_iic_info_struct;

/*==================================================================================================================*/
/* =============== 内部协议API (通常用户无需直接调用) =============== */
/*==================================================================================================================*/

void soft_iic_start(soft_iic_info_struct *soft_iic_obj);
void soft_iic_stop(soft_iic_info_struct *soft_iic_obj);
void soft_iic_send_ack(soft_iic_info_struct *soft_iic_obj, uint8_t ack);
uint8_t soft_iic_wait_ack(soft_iic_info_struct *soft_iic_obj);
uint8_t soft_iic_send_data(soft_iic_info_struct *soft_iic_obj, uint8_t dat);
uint8_t soft_iic_read_data(soft_iic_info_struct *soft_iic_obj, uint8_t ack);

/*==================================================================================================================*/
/* =============== 基础操作API =============== */
/*==================================================================================================================*/

/**
 * @brief       写8位数据到I2C设备
 * @param   *soft_iic_obj   I2C配置对象指针
 * @param   dat             要写的数据
 */
void soft_iic_write_8bit(soft_iic_info_struct *soft_iic_obj, const uint8_t dat);

/**
 * @brief       写8位数组到I2C设备
 * @param   *soft_iic_obj   I2C配置对象指针
 * @param   *dat            数据缓冲区指针
 * @param   len             数据长度
 */
void soft_iic_write_8bit_array(soft_iic_info_struct *soft_iic_obj, const uint8_t *dat, uint32_t len);

/**
 * @brief       从I2C设备读取8位数据
 * @param   *soft_iic_obj   I2C配置对象指针
 * @return  读取的数据
 */
uint8_t soft_iic_read_8bit(soft_iic_info_struct *soft_iic_obj);

/**
 * @brief       从I2C设备读取8位数组
 * @param   *soft_iic_obj   I2C配置对象指针
 * @param   *dat            数据缓冲区指针
 * @param   len             数据长度
 */
void soft_iic_read_8bit_array(soft_iic_info_struct *soft_iic_obj, uint8_t *dat, uint32_t len);

/*==================================================================================================================*/
/* =============== 寄存器操作API =============== */
/*==================================================================================================================*/

/**
 * @brief       写8位寄存器
 * @param   *soft_iic_obj   I2C配置对象指针
 * @param   register_name   寄存器地址
 * @param   dat             要写的数据
 */
void soft_iic_write_8bit_register(soft_iic_info_struct *soft_iic_obj, const uint8_t register_name, const uint8_t dat);

/**
 * @brief       写8位寄存器数组
 * @param   *soft_iic_obj   I2C配置对象指针
 * @param   register_name   寄存器地址
 * @param   *dat            数据缓冲区指针
 * @param   len             数据长度
 */
void soft_iic_write_8bit_registers(soft_iic_info_struct *soft_iic_obj, const uint8_t register_name, const uint8_t *dat, uint32_t len);

/**
 * @brief       读8位寄存器
 * @param   *soft_iic_obj   I2C配置对象指针
 * @param   register_name   寄存器地址
 * @return  读取的数据
 */
uint8_t soft_iic_read_8bit_register(soft_iic_info_struct *soft_iic_obj, const uint8_t register_name);

/**
 * @brief       读8位寄存器数组
 * @param   *soft_iic_obj   I2C配置对象指针
 * @param   register_name   寄存器地址
 * @param   *dat            数据缓冲区指针
 * @param   len             数据长度
 */
void soft_iic_read_8bit_registers(soft_iic_info_struct *soft_iic_obj, const uint8_t register_name, uint8_t *dat, uint32_t len);

/*==================================================================================================================*/
/* =============== 高级传输API =============== */
/*==================================================================================================================*/

/**
 * @brief       I2C传输8位数组 (先写后读，适用于需要先发送命令再读取数据的场景)
 * @param   *soft_iic_obj   I2C配置对象指针
 * @param   *write_data     要写的数据缓冲区
 * @param   write_len       写数据长度
 * @param   *read_data      读取数据缓冲区
 * @param   read_len        读数据长度
 */
void soft_iic_transfer_8bit_array(soft_iic_info_struct *soft_iic_obj, const uint8_t *write_data, uint32_t write_len,
                                   uint8_t *read_data, uint32_t read_len);

/*==================================================================================================================*/
/* =============== 初始化API =============== */
/*==================================================================================================================*/

/**
 * @brief       初始化软件I2C
 * @param   *soft_iic_obj   I2C配置对象指针
 * @param   addr            I2C设备地址 (7位地址模式)
 * @param   delay           I2C延时时钟 (数值越大I2C速度越慢)
 * @param   scl_port         SCL端口 (GPIOA/GPIOB/GPIOC...)
 * @param   scl_pin         SCL引脚
 * @param   sda_port         SDA端口
 * @param   sda_pin         SDA引脚
 * @note        一次性完成所有初始化，包括GPIO配置
 */
void soft_iic_init(soft_iic_info_struct *soft_iic_obj, uint8_t addr, uint32_t delay,
                   GPIO_TypeDef *scl_port, gpio_pin_enum scl_pin,
                   GPIO_TypeDef *sda_port, gpio_pin_enum sda_pin);

#endif /* __MYIIC_H */
