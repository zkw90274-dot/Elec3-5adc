/*********************************************************************************************************************
 * @file        ui.h
 * @brief       OLED UI 显示界面头文件
 * @platform    STC32G
 * @chip        SSD1306 (128x64 OLED)
 *
 * @功能说明:
 *  - 提供常用的 OLED 显示界面函数
 *  - 支持标题、数据、状态栏显示
 *  - PID 参数调节菜单
********************************************************************************************************************/

#ifndef __UI_H
#define __UI_H

#include "zf_common_headfile.h"

/*==================================================================================================================*/
/* =============== 显示区域定义 =============== */
/*==================================================================================================================*/

/** OLED显示区域划分 */
#define UI_TITLE_LINE        1    // 标题行
#define UI_DATA_START_LINE   2    // 数据起始行
#define UI_DATA_END_LINE     3    // 数据结束行
#define UI_STATUS_LINE       4    // 状态行

/** 显示宽度 */
#define UI_FULL_WIDTH        16   // 一行最多显示16个字符 (8x16字体)

/*==================================================================================================================*/
/* =============== 菜单状态定义 =============== */
/*==================================================================================================================*/

/** 菜单状态 */
typedef enum
{
    UI_MENU_NORMAL = 0,      // 普通显示模式
    UI_MENU_EDIT,            // 编辑模式
    UI_MENU_SAVE             // 保存提示
} ui_menu_state_t;

/** PID 参数项 */
typedef enum
{
    UI_PID_LEFT_KP = 0,      // 左电机 Kp
    UI_PID_LEFT_KI,          // 左电机 Ki
    UI_PID_LEFT_KD,          // 左电机 Kd
    UI_PID_RIGHT_KP,         // 右电机 Kp
    UI_PID_RIGHT_KI,         // 右电机 Ki
    UI_PID_RIGHT_KD,         // 右电机 Kd
    UI_PID_COUNT             // 参数总数
} ui_pid_item_t;

/** 菜单上下文 */
typedef struct
{
    ui_menu_state_t state;   // 菜单状态
    ui_pid_item_t item;      // 当前选中的参数项
    float *value;            // 当前参数值指针
    uint8 blink_cnt;         // 闪烁计数器
    uint8 save_cnt;          // 保存提示计数器
} ui_menu_t;

/** 菜单配置 */
#define MENU_BLINK_PERIOD    10       // 闪烁周期 (100ms)
#define MENU_SAVE_DELAY       20       // 保存提示显示时间 (2秒)
#define MENU_STEP             0.01f    // 步进值

/*==================================================================================================================*/
/* =============== UI 基础函数 =============== */
/*==================================================================================================================*/

/**
 * @brief       UI初始化（包含OLED初始化）
 */
void UI_Init(void);

/**
 * @brief       显示标题
 * @param   title   标题字符串
 */
void UI_ShowTitle(char *title);

/**
 * @brief       显示数据行
 * @param   line    行号 (2~3)
 * @param   label   数据标签
 * @param   value   数据值
 * @param   unit    单位
 */
void UI_ShowData(uint8 line, char *label, float value, char *unit);

/**
 * @brief       显示整数数据
 * @param   line    行号 (1~4)
 * @param   label   数据标签
 * @param   value   整数值
 * @param   unit    单位
 */
void UI_ShowInt(uint8 line, char *label, int32 value, char *unit);

/**
 * @brief       显示状态栏信息
 * @param   status  状态字符串
 */
void UI_ShowStatus(char *status);

/**
 * @brief       清空数据区域（行2-3）
 */
void UI_ClearDataArea(void);

/**
 * @brief       显示完整界面（标题+数据+状态）
 * @param   title   标题
 * @param   line2   第2行数据
 * @param   line3   第3行数据
 * @param   status  状态信息
 */
void UI_ShowFullDisplay(char *title, char *line2, char *line3, char *status);

/**
 * @brief       OLED简单测试界面
 */
void UI_Test(void);

/*==================================================================================================================*/
/* =============== 菜单函数 =============== */
/*==================================================================================================================*/

/**
 * @brief       菜单初始化
 */
void UI_MenuInit(void);

/**
 * @brief       菜单更新函数 (在主循环中调用)
 */
void UI_MenuUpdate(void);

/**
 * @brief       进入编辑模式
 */
void UI_MenuEnterEdit(void);

/**
 * @brief       退出编辑模式
 */
void UI_MenuExitEdit(void);

/**
 * @brief       保存当前参数到 EEPROM
 */
void UI_MenuSave(void);

/**
 * @brief       增加当前参数值
 * @param   step    步进值
 */
void UI_MenuIncrease(float step);

/**
 * @brief       减少当前参数值
 * @param   step    步进值
 */
void UI_MenuDecrease(float step);

/**
 * @brief       切换到下一个参数项
 */
void UI_MenuNextItem(void);

/**
 * @brief       获取参数名称
 * @param   item    参数项
 * @return  参数名称字符串
 */
char* UI_MenuGetItemName(ui_pid_item_t item);

/**
 * @brief       获取参数值指针
 * @param   item    参数项
 * @return  参数值指针
 */
float* UI_MenuGetItemValue(ui_pid_item_t item);

#endif /* __UI_H */
