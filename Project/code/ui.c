/*********************************************************************************************************************
 * @file        ui.c
 * @brief       OLED UI 显示界面实现 + PID 参数调节菜单
 * @platform    STC32G
 * @chip        SSD1306 (128x64 OLED)
********************************************************************************************************************/

#include "oled.h"
#include "ui.h"
#include "pid.h"
#include "myeeprom.h"
#include "key.h"

/*==================================================================================================================*/
/* =============== 私有变量 =============== */
/*==================================================================================================================*/

static ui_menu_t menu;

// 参数名称数组
static char* item_names[UI_PID_COUNT];

/*==================================================================================================================*/
/* =============== 私有函数声明 =============== */
/*==================================================================================================================*/

static void init_item_names(void);
static void display_menu(void);
static void display_menu_normal(void);
static void display_menu_edit(void);
static void display_menu_save(void);

/*==================================================================================================================*/
/* =============== UI 基础函数 =============== */
/*==================================================================================================================*/

/**
 * @brief       UI初始化（包含OLED初始化）
 */
void UI_Init(void)
{
    OLED_Init();
    OLED_Clear();
}

/**
 * @brief       显示标题
 * @param   title   标题字符串
 */
void UI_ShowTitle(char *title)
{
    uint8 i;
    uint8 len;
    uint8 start_col;
    char *p;

    // 清空标题行
    for(i = 1; i <= UI_FULL_WIDTH; i++)
    {
        OLED_ShowChar(UI_TITLE_LINE, i, ' ');
    }
    // 显示标题（居中显示）
    len = 0;
    p = title;
    while(*p && len < UI_FULL_WIDTH)
    {
        len++;
        p++;
    }
    start_col = (UI_FULL_WIDTH - len) / 2 + 1;
    if(start_col < 1) start_col = 1;
    OLED_ShowString(UI_TITLE_LINE, start_col, title);
}

/**
 * @brief       显示数据行
 * @param   line    行号 (2~3)
 * @param   label   数据标签
 * @param   value   数据值
 * @param   unit    单位
 */
void UI_ShowData(uint8 line, char *label, float value, char *unit)
{
    char buf[20];
    uint8 i;
    uint8 len;
    char *p;
    int32 intval;
    char temp[10];
    uint8 tlen;

    // 清空该行
    for(i = 1; i <= UI_FULL_WIDTH; i++)
    {
        OLED_ShowChar(line, i, ' ');
    }

    // 显示数据: "Label: 123.4 Unit"
    len = 0;
    p = label;
    while(*p && len < 8)
    {
        buf[len++] = *p++;
    }
    buf[len++] = ':';
    buf[len++] = ' ';

    // 转换浮点数为字符串
    if(value >= 0)
    {
        buf[len++] = ' ';
    }

    // 简单整数转换（为节省代码空间）
    intval = (int32)value;
    if(intval < 0)
    {
        buf[len++] = '-';
        intval = -intval;
    }

    // 转换为字符串
    tlen = 0;
    if(intval == 0)
    {
        temp[tlen++] = '0';
    }
    else
    {
        while(intval > 0)
        {
            temp[tlen++] = (intval % 10) + '0';
            intval /= 10;
        }
    }

    // 反转并复制
    for(i = 0; i < tlen && len < 15; i++)
    {
        buf[len++] = temp[tlen - 1 - i];
    }

    // 添加单位
    p = unit;
    while(*p && len < 16)
    {
        buf[len++] = *p++;
    }
    buf[len] = '\0';

    OLED_ShowString(line, 1, buf);
}

/**
 * @brief       显示整数数据
 * @param   line    行号 (1~4)
 * @param   label   数据标签
 * @param   value   整数值
 * @param   unit    单位
 */
void UI_ShowInt(uint8 line, char *label, int32 value, char *unit)
{
    UI_ShowData(line, label, (float)value, unit);
}

/**
 * @brief       显示状态栏信息
 * @param   status  状态字符串
 */
void UI_ShowStatus(char *status)
{
    uint8 i;

    // 清空状态行
    for(i = 1; i <= UI_FULL_WIDTH; i++)
    {
        OLED_ShowChar(UI_STATUS_LINE, i, ' ');
    }
    // 显示状态（左对齐）
    OLED_ShowString(UI_STATUS_LINE, 1, status);
}

/**
 * @brief       清空数据区域（行2-3）
 */
void UI_ClearDataArea(void)
{
    uint8 line;
    uint8 col;

    for(line = UI_DATA_START_LINE; line <= UI_DATA_END_LINE; line++)
    {
        for(col = 1; col <= UI_FULL_WIDTH; col++)
        {
            OLED_ShowChar(line, col, ' ');
        }
    }
}

/**
 * @brief       显示完整界面（标题+数据+状态）
 * @param   title   标题
 * @param   line2   第2行数据
 * @param   line3   第3行数据
 * @param   status  状态信息
 */
void UI_ShowFullDisplay(char *title, char *line2, char *line3, char *status)
{
    UI_ShowTitle(title);
    OLED_ShowString(UI_DATA_START_LINE, 1, line2);
    OLED_ShowString(UI_DATA_START_LINE + 1, 1, line3);
    UI_ShowStatus(status);
}

/**
 * @brief       OLED简单测试界面
 */
void UI_Test(void)
{
    uint16 i;

    UI_Init();

    // 显示测试界面
    UI_ShowTitle("OLED TEST");

    OLED_ShowString(2, 1, "Line 2 Test");
    OLED_ShowString(3, 1, "Line 3 Test");

    OLED_ShowString(4, 1, "Status: OK");

    // 显示一些数字测试
    system_delay_ms(2000);

    OLED_Clear();

    // 显示计数器测试
    for(i = 0; i < 100; i++)
    {
        UI_ShowTitle("Counter");
        OLED_ShowNum(2, 6, i, 3);
        UI_ShowStatus("Running...");
        system_delay_ms(100);
    }

    OLED_Clear();
    OLED_ShowString(2, 4, "Test Done!");
}

/*==================================================================================================================*/
/* =============== 菜单函数 =============== */
/*==================================================================================================================*/

/**
 * @brief       菜单初始化
 */
void UI_MenuInit(void)
{
    init_item_names();

    menu.state = UI_MENU_NORMAL;
    menu.item = UI_PID_LEFT_KP;
    menu.value = UI_MenuGetItemValue(menu.item);
    menu.blink_cnt = 0;
    menu.save_cnt = 0;
}

/**
 * @brief       菜单更新函数 (在主循环中调用)
 */
void UI_MenuUpdate(void)
{
    // 更新闪烁计数器
    menu.blink_cnt++;

    // 更新保存提示计数器
    if(menu.save_cnt > 0)
    {
        menu.save_cnt--;
        if(menu.save_cnt == 0)
        {
            menu.state = UI_MENU_NORMAL;
        }
    }

    // 按键处理
    Key_Disp();

    if(menu.state == UI_MENU_NORMAL)
    {
        // 普通模式：按键1进入编辑模式
        if(key_down == KEY1)
        {
            UI_MenuEnterEdit();
        }
    }
    else if(menu.state == UI_MENU_EDIT)
    {
        // 编辑模式
        if(key_down == KEY1)
        {
            // 切换到下一个参数
            UI_MenuNextItem();
        }
        else if(key_down == KEY2)
        {
            // 减少参数
            UI_MenuDecrease(MENU_STEP);
            // 保存到 EEPROM
            UI_MenuSave();
        }
        else if(key_down == KEY3)
        {
            // 增加参数
            UI_MenuIncrease(MENU_STEP);
            // 保存到 EEPROM
            UI_MenuSave();
        }
        else if(key_down == KEY4)
        {
            // 退出编辑模式
            UI_MenuExitEdit();
        }
    }

    // 显示菜单
    display_menu();
}

/**
 * @brief       进入编辑模式
 */
void UI_MenuEnterEdit(void)
{
    menu.state = UI_MENU_EDIT;
    menu.item = UI_PID_LEFT_KP;
    menu.value = UI_MenuGetItemValue(menu.item);
    menu.blink_cnt = 0;
}

/**
 * @brief       退出编辑模式
 */
void UI_MenuExitEdit(void)
{
    menu.state = UI_MENU_NORMAL;
    // 保存参数到 EEPROM
    UI_MenuSave();
}

/**
 * @brief       保存当前参数到 EEPROM
 */
void UI_MenuSave(void)
{
    myeeprom_save_speed_pid(&pid_motor_left, &pid_motor_right);
    menu.state = UI_MENU_SAVE;
    menu.save_cnt = MENU_SAVE_DELAY;
}

/**
 * @brief       增加当前参数值
 * @param   step    步进值
 */
void UI_MenuIncrease(float step)
{
    if(menu.value != 0)
    {
        *menu.value += step;
        // 限制范围
        if(*menu.value > 10.0f)
        {
            *menu.value = 10.0f;
        }
    }
}

/**
 * @brief       减少当前参数值
 * @param   step    步进值
 */
void UI_MenuDecrease(float step)
{
    if(menu.value != 0)
    {
        *menu.value -= step;
        // 限制范围
        if(*menu.value < 0.0f)
        {
            *menu.value = 0.0f;
        }
    }
}

/**
 * @brief       切换到下一个参数项
 */
void UI_MenuNextItem(void)
{
    menu.item++;
    if(menu.item >= UI_PID_COUNT)
    {
        menu.item = UI_PID_LEFT_KP;
    }
    menu.value = UI_MenuGetItemValue(menu.item);
    menu.blink_cnt = 0;
}

/**
 * @brief       获取参数名称
 * @param   item    参数项
 * @return  参数名称字符串
 */
char* UI_MenuGetItemName(ui_pid_item_t item)
{
    if(item < UI_PID_COUNT)
    {
        return item_names[item];
    }
    return "???";
}

/**
 * @brief       获取参数值指针
 * @param   item    参数项
 * @return  参数值指针
 */
float* UI_MenuGetItemValue(ui_pid_item_t item)
{
    float *result;

    switch(item)
    {
        case UI_PID_LEFT_KP:  result = &pid_motor_left.kp; break;
        case UI_PID_LEFT_KI:  result = &pid_motor_left.ki; break;
        case UI_PID_LEFT_KD:  result = &pid_motor_left.kd; break;
        case UI_PID_RIGHT_KP: result = &pid_motor_right.kp; break;
        case UI_PID_RIGHT_KI: result = &pid_motor_right.ki; break;
        case UI_PID_RIGHT_KD: result = &pid_motor_right.kd; break;
        default: result = 0; break;
    }
    return result;
}

/*==================================================================================================================*/
/* =============== 私有函数实现 =============== */
/*==================================================================================================================*/

/**
 * @brief       初始化参数名称数组
 */
static void init_item_names(void)
{
    item_names[UI_PID_LEFT_KP]  = "L-kp";
    item_names[UI_PID_LEFT_KI]  = "L-ki";
    item_names[UI_PID_LEFT_KD]  = "L-kd";
    item_names[UI_PID_RIGHT_KP] = "R-kp";
    item_names[UI_PID_RIGHT_KI] = "R-ki";
    item_names[UI_PID_RIGHT_KD] = "R-kd";
}

/**
 * @brief       显示菜单界面
 */
static void display_menu(void)
{
    if(menu.state == UI_MENU_SAVE)
    {
        display_menu_save();
    }
    else if(menu.state == UI_MENU_EDIT)
    {
        display_menu_edit();
    }
    else
    {
        display_menu_normal();
    }
}

/**
 * @brief       显示普通模式界面
 */
static void display_menu_normal(void)
{
    // 清屏
    OLED_Clear();

    // 标题
    OLED_ShowString(1, 1, "PID Menu");
    OLED_ShowString(1, 10, "K1=Edit");

    // 显示左右电机参数
    OLED_ShowString(2, 1, "L:");
    OLED_ShowNum(2, 3, (int32)(pid_motor_left.kp * 100), 4);

    OLED_ShowString(3, 1, "R:");
    OLED_ShowNum(3, 3, (int32)(pid_motor_right.kp * 100), 4);
}

/**
 * @brief       显示编辑模式界面
 */
static void display_menu_edit(void)
{
    uint8 show_cursor;
    float value;
    int32 display_val;

    // 清屏
    OLED_Clear();

    // 标题
    OLED_ShowString(1, 1, "PID Edit");
    OLED_ShowString(1, 10, "<>Adj");

    // 显示当前选中的参数
    show_cursor = ((menu.blink_cnt / MENU_BLINK_PERIOD) % 2) == 0;

    // 当前选中项
    OLED_ShowString(2, 1, ">");
    OLED_ShowString(2, 2, UI_MenuGetItemName(menu.item));
    OLED_ShowString(2, 7, ":");

    // 显示参数值 (x100 显示两位小数)
    value = *menu.value;
    display_val = (int32)(value * 100);
    if(show_cursor)
    {
        OLED_ShowSignedNum(2, 8, display_val, 4);
    }

    // 显示下一个参数提示
    if(menu.item + 1 < UI_PID_COUNT)
    {
        OLED_ShowString(4, 1, "Next:");
        OLED_ShowString(4, 6, UI_MenuGetItemName(menu.item + 1));
    }
}

/**
 * @brief       显示保存成功提示
 */
static void display_menu_save(void)
{
    OLED_Clear();
    OLED_ShowString(2, 4, "Saved!");
}
