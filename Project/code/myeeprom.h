/*********************************************************************************************************************
 * @file        myeeprom.h
 * @brief       EEPROM 存储模块 - 速度环 PID 参数存储
 * @platform    STC32G
 * @note        用于存储和加载电机速度环 PID 参数
********************************************************************************************************************/

#ifndef __MYEEPROM_H
#define __MYEEPROM_H

#include "zf_common_headfile.h"

/*==================================================================================================================*/
/* =============== EEPROM 地址定义 =============== */
/*==================================================================================================================*/

#define EEPROM_PID_ADDR          0x0000    // PID参数存储起始地址

#define EEPROM_PAGE_SIZE         512       // EEPROM 页大小 (字节)

#define EEPROM_MAGIC_ADDR        0x0000    // 魔数地址 (用于检测数据有效性)
#define EEPROM_MAGIC_VALUE       0xA5A5    // 魔数值 (如果读取到这个值说明数据有效)

/*==================================================================================================================*/
/* =============== 数据结构定义 =============== */
/*==================================================================================================================*/

/**
 * @brief       速度环 PID 参数存储结构体
 */
typedef struct
{
    // 左电机 PID 参数
    float left_kp;
    float left_ki;
    float left_kd;

    // 右电机 PID 参数
    float right_kp;
    float right_ki;
    float right_kd;

    // 魔数 (用于检测数据有效性)
    uint16 magic;

} speed_pid_storage_t;

/*==================================================================================================================*/
/* =============== API 函数声明 =============== */
/*==================================================================================================================*/

/**
 * @brief       EEPROM 存储模块初始化
 * @note        初始化 IAP 功能
 */
void myeeprom_init(void);

/**
 * @brief       保存速度环 PID 参数到 EEPROM
 * @param       left_pid    左电机 PID 结构体指针
 * @param       right_pid   右电机 PID 结构体指针
 * @return      0=成功, 1=失败
 */
uint8 myeeprom_save_speed_pid(pid_param_t *left_pid, pid_param_t *right_pid);

/**
 * @brief       从 EEPROM 加载速度环 PID 参数
 * @param       left_pid    左电机 PID 结构体指针
 * @param       right_pid   右电机 PID 结构体指针
 * @return      0=成功加载, 1=数据无效(使用默认值)
 * @note        如果 EEPROM 数据无效，会使用默认值初始化 PID
 */
uint8 myeeprom_load_speed_pid(pid_param_t *left_pid, pid_param_t *right_pid);

/**
 * @brief       恢复默认 PID 参数并保存到 EEPROM
 * @param       left_pid    左电机 PID 结构体指针
 * @param       right_pid   右电机 PID 结构体指针
 * @note        默认值: kp=1.7, ki=0.22, kd=0.2
 */
void myeeprom_restore_default(pid_param_t *left_pid, pid_param_t *right_pid);

/**
 * @brief       擦除 EEPROM 存储区域
 * @note        擦除 PID 参数所在的页 (512字节)
 */
void myeeprom_erase(void);

#endif /* __MYEEPROM_H */
