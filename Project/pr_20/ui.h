/*********************************************************************************************************************
 * @file        ui.h
 * @brief       OLED PID 参数调节菜单界面 (STM32F103 适配版)
 * @platform    STM32F103C8T6
 * @chip        SSD1306 (128x64 OLED)
 *
 * @功能说明:
 *  - 四个按键控制 (KEY1/KEY2/KEY3/KEY4)
 *  - 三种显示模式：普通、编辑、保存提示
 *  - 六个 PID 参数 (L-kp, L-ki, L-kd, R-kp, R-ki, R-kd)
********************************************************************************************************************/

#ifndef __UI_H
#define __UI_H

#include "stm32f10x.h"

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

/** 菜单配置 */
#define MENU_BLINK_PERIOD    10       // 闪烁周期 (100ms, 10次调用)
#define MENU_SAVE_DELAY      20       // 保存提示显示时间 (2秒, 20次调用)
#define MENU_STEP            0.01f    // 步进值

/*==================================================================================================================*/
/* =============== UI 函数 =============== */
/*==================================================================================================================*/

/**
 * @brief   UI初始化（包含OLED初始化）
 */
void UI_Init(void);

/**
 * @brief   菜单初始化
 */
void UI_MenuInit(void);

/**
 * @brief   菜单更新函数 (在主循环中调用, 10ms周期)
 * @note    内部会自动调用 Key_Disp() 和 OLED显示函数
 */
void UI_MenuUpdate(void);

/**
 * @brief   进入编辑模式
 */
void UI_MenuEnterEdit(void);

/**
 * @brief   退出编辑模式
 */
void UI_MenuExitEdit(void);

/**
 * @brief   增加当前参数值
 * @param   step    步进值
 */
void UI_MenuIncrease(float step);

/**
 * @brief   减少当前参数值
 * @param   step    步进值
 */
void UI_MenuDecrease(float step);

/**
 * @brief   切换到下一个参数项
 */
void UI_MenuNextItem(void);

/**
 * @brief   获取参数名称
 * @param   item    参数项
 * @return  参数名称字符串
 */
char* UI_MenuGetItemName(ui_pid_item_t item);

#endif /* __UI_H */
