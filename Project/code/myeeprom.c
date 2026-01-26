/*********************************************************************************************************************
 * @file        myeeprom.c
 * @brief       EEPROM 存储模块 - 速度环 PID 参数存储实现
 * @platform    STC32G
********************************************************************************************************************/

#include "myeeprom.h"
#include "zf_driver_eeprom.h"
#include "pid.h"

/*==================================================================================================================*/
/* =============== 私有变量 =============== */
/*==================================================================================================================*/

// 默认 PID 参数值
static const float DEFAULT_KP = 1.7f;
static const float DEFAULT_KI = 0.22f;
static const float DEFAULT_KD = 0.2f;

/*==================================================================================================================*/
/* =============== 私有函数声明 =============== */
/*==================================================================================================================*/

/**
 * @brief       浮点数写入 EEPROM
 * @param       addr    EEPROM 地址
 * @param       value   浮点数值
 */
static void write_float(uint32 addr, float value);

/**
 * @brief       从 EEPROM 读取浮点数
 * @param       addr    EEPROM 地址
 * @return      读取到的浮点数值
 */
static float read_float(uint32 addr);

/*==================================================================================================================*/
/* =============== API 函数实现 =============== */
/*==================================================================================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     EEPROM 存储模块初始化
// 参数说明     void
// 返回参数     void
// 使用示例     myeeprom_init();
// 备注信息     初始化 IAP 功能
//-------------------------------------------------------------------------------------------------------------------
void myeeprom_init(void)
{
    iap_init();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     保存速度环 PID 参数到 EEPROM
// 参数说明     left_pid    左电机 PID 结构体指针
// 参数说明     right_pid   右电机 PID 结构体指针
// 返回参数     uint8       0=成功, 1=失败
// 使用示例     myeeprom_save_speed_pid(&pid_motor_left, &pid_motor_right);
// 备注信息     将左右电机的 Kp, Ki, Kd 参数保存到 EEPROM
//-------------------------------------------------------------------------------------------------------------------
uint8 myeeprom_save_speed_pid(pid_param_t *left_pid, pid_param_t *right_pid)
{
    uint8 magic_low;
    uint8 magic_high;

    if(left_pid == 0 || right_pid == 0)
    {
        return 1;
    }

    // 写入左电机 PID 参数 (跳过魔数位置，先写数据)
    write_float(EEPROM_PID_ADDR + 2, left_pid->kp);
    write_float(EEPROM_PID_ADDR + 6, left_pid->ki);
    write_float(EEPROM_PID_ADDR + 10, left_pid->kd);

    // 写入右电机 PID 参数
    write_float(EEPROM_PID_ADDR + 14, right_pid->kp);
    write_float(EEPROM_PID_ADDR + 18, right_pid->ki);
    write_float(EEPROM_PID_ADDR + 22, right_pid->kd);

    // 写入魔数 (最后写入，确保数据完整性)
    magic_low = (uint8)(EEPROM_MAGIC_VALUE & 0xFF);
    magic_high = (uint8)((EEPROM_MAGIC_VALUE >> 8) & 0xFF);
    iap_write_byte(EEPROM_MAGIC_ADDR, magic_low);
    iap_write_byte(EEPROM_MAGIC_ADDR + 1, magic_high);

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从 EEPROM 加载速度环 PID 参数
// 参数说明     left_pid    左电机 PID 结构体指针
// 参数说明     right_pid   右电机 PID 结构体指针
// 返回参数     uint8       0=成功加载, 1=数据无效(使用默认值)
// 使用示例     myeeprom_load_speed_pid(&pid_motor_left, &pid_motor_right);
// 备注信息     读取 EEPROM 中的 PID 参数，如果魔数不匹配则使用默认值
//-------------------------------------------------------------------------------------------------------------------
uint8 myeeprom_load_speed_pid(pid_param_t *left_pid, pid_param_t *right_pid)
{
    uint8 magic_low;
    uint8 magic_high;
    uint16 magic_read;

    if(left_pid == 0 || right_pid == 0)
    {
        return 1;
    }

    // 读取魔数验证数据有效性
    magic_low = iap_read_byte(EEPROM_MAGIC_ADDR);
    magic_high = iap_read_byte(EEPROM_MAGIC_ADDR + 1);
    magic_read = magic_low | (magic_high << 8);

    if(magic_read != EEPROM_MAGIC_VALUE)
    {
        // 数据无效，使用默认值
        myeeprom_restore_default(left_pid, right_pid);
        return 1;
    }

    // 数据有效，读取 PID 参数 (跳过魔数 2字节)
    left_pid->kp = read_float(EEPROM_PID_ADDR + 2);
    left_pid->ki = read_float(EEPROM_PID_ADDR + 6);
    left_pid->kd = read_float(EEPROM_PID_ADDR + 10);

    // 读取右电机 PID 参数
    right_pid->kp = read_float(EEPROM_PID_ADDR + 14);
    right_pid->ki = read_float(EEPROM_PID_ADDR + 18);
    right_pid->kd = read_float(EEPROM_PID_ADDR + 22);

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     恢复默认 PID 参数并保存到 EEPROM
// 参数说明     left_pid    左电机 PID 结构体指针
// 参数说明     right_pid   右电机 PID 结构体指针
// 返回参数     void
// 使用示例     myeeprom_restore_default(&pid_motor_left, &pid_motor_right);
// 备注信息     使用默认值初始化 PID 并保存到 EEPROM
//-------------------------------------------------------------------------------------------------------------------
void myeeprom_restore_default(pid_param_t *left_pid, pid_param_t *right_pid)
{
    if(left_pid != 0)
    {
        pid_init(left_pid, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    }

    if(right_pid != 0)
    {
        pid_init(right_pid, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    }

    // 保存默认值到 EEPROM
    myeeprom_save_speed_pid(left_pid, right_pid);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     擦除 EEPROM 存储区域
// 参数说明     void
// 返回参数     void
// 使用示例     myeeprom_erase();
// 备注信息     擦除 PID 参数所在的页 (512字节)
//-------------------------------------------------------------------------------------------------------------------
void myeeprom_erase(void)
{
    iap_erase_page(EEPROM_PID_ADDR);
}

/*==================================================================================================================*/
/* =============== 私有函数实现 =============== */
/*==================================================================================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     浮点数写入 EEPROM
// 参数说明     addr    EEPROM 地址
// 参数说明     value   浮点数值
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
static void write_float(uint32 addr, float value)
{
    uint8 *p;
    uint8 i;

    p = (uint8 *)&value;

    for(i = 0; i < 4; i++)
    {
        iap_write_byte(addr + i, p[i]);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从 EEPROM 读取浮点数
// 参数说明     addr    EEPROM 地址
// 返回参数     float   读取到的浮点数值
//-------------------------------------------------------------------------------------------------------------------
static float read_float(uint32 addr)
{
    float value;
    uint8 *p;
    uint8 i;

    p = (uint8 *)&value;

    for(i = 0; i < 4; i++)
    {
        p[i] = iap_read_byte(addr + i);
    }

    return value;
}
