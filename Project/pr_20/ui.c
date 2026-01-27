/*********************************************************************************************************************
 * @file        ui.c
 * @brief       OLED PID 参数调节菜单界面实现 (STM32F103 适配版)
 * @platform    STM32F103C8T6
 * @chip        SSD1306 (128x64 OLED)
********************************************************************************************************************/

#include "ui.h"
#include "OLED.h"
#include "Key.h"
#include "Delay.h"

/*==================================================================================================================*/
/* =============== 私有变量 =============== */
/*==================================================================================================================*/

/** 菜单上下文 */
static struct
{
    ui_menu_state_t state;   // 菜单状态
    ui_menu_state_t last_state;  // 上一次的状态 (用于检测状态变化)
    ui_pid_item_t item;      // 当前选中的参数项
    float value;             // 当前参数值
    uint8_t blink_cnt;       // 闪烁计数器
    uint8_t blink_state;     // 当前闪烁状态 (0/1)
    uint8_t save_cnt;        // 保存提示计数器
} menu;

/** 模拟的 PID 参数数据 (静态存储，不需要 EEPROM) */
static struct
{
    float kp;
    float ki;
    float kd;
} pid_left = {1.20f, 0.50f, 0.30f};    // 左电机默认值

static struct
{
    float kp;
    float ki;
    float kd;
} pid_right = {1.50f, 0.60f, 0.25f};   // 右电机默认值

/** 参数名称数组 */
static char* item_names[UI_PID_COUNT] = {
    "Lp",    // UI_PID_LEFT_KP
    "Li",    // UI_PID_LEFT_KI
    "Ld",    // UI_PID_LEFT_KD
    "Rp",    // UI_PID_RIGHT_KP
    "Ri",    // UI_PID_RIGHT_KI
    "Rd"     // UI_PID_RIGHT_KD
};

/*==================================================================================================================*/
/* =============== 私有函数声明 =============== */
/*==================================================================================================================*/

static void display_menu_normal(void);
static void display_menu_edit(void);
static void display_menu_save(void);
static void display_edit_value(uint8_t show);
static float* get_pid_value_ptr(ui_pid_item_t item);

/*==================================================================================================================*/
/* =============== UI 基础函数 =============== */
/*==================================================================================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UI初始化
// 参数说明     void
// 返回参数     void
// 使用示例     UI_Init();
// 备注信息     包含OLED初始化
//-------------------------------------------------------------------------------------------------------------------
void UI_Init(void)
{
    OLED_Init();
    OLED_Clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     菜单初始化
// 参数说明     void
// 返回参数     void
// 使用示例     UI_MenuInit();
// 备注信息     初始化菜单状态和参数
//-------------------------------------------------------------------------------------------------------------------
void UI_MenuInit(void)
{
    key_init();  // 初始化按键 GPIO

    menu.state = UI_MENU_NORMAL;
    menu.last_state = UI_MENU_NORMAL;
    menu.item = UI_PID_LEFT_KP;
    menu.value = *get_pid_value_ptr(menu.item);
    menu.blink_cnt = 0;
    menu.blink_state = 0;
    menu.save_cnt = 0;

    // 初始显示
    display_menu_normal();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     菜单更新函数 (在主循环中调用)
// 参数说明     void
// 返回参数     void
// 使用示例     UI_MenuUpdate();  // 在while(1)中循环调用
// 备注信息     内部会自动调用 Key_Disp() 和 OLED显示函数
//              建议调用周期: 10ms
//-------------------------------------------------------------------------------------------------------------------
void UI_MenuUpdate(void)
{
    uint8_t need_redraw = 0;
    uint8_t new_blink_state;

    // 更新闪烁计数器
    menu.blink_cnt++;

    // 计算新的闪烁状态 (每100ms切换一次)
    new_blink_state = (menu.blink_cnt / MENU_BLINK_PERIOD) % 2;

    // 更新保存提示计数器
    if(menu.save_cnt > 0)
    {
        menu.save_cnt--;
        if(menu.save_cnt == 0)
        {
            menu.state = UI_MENU_NORMAL;
            need_redraw = 1;
        }
    }

    // 按键处理
    Key_Disp();

    // 状态机处理
    if(menu.state == UI_MENU_NORMAL)
    {
        // 普通模式：KEY1 进入编辑模式
        if(key_down == KEY1)
        {
            UI_MenuEnterEdit();
            need_redraw = 1;
        }
    }
    else if(menu.state == UI_MENU_EDIT)
    {
        // 编辑模式
        if(key_down == KEY1)
        {
            // 切换到下一个参数 (需要完整重绘)
            UI_MenuNextItem();
            need_redraw = 1;
        }
        else if(key_down == KEY2)
        {
            // 减少参数 (只更新数值，不清屏)
            UI_MenuDecrease(MENU_STEP);
            display_edit_value(1);  // 直接显示新数值
        }
        else if(key_down == KEY3)
        {
            // 增加参数 (只更新数值，不清屏)
            UI_MenuIncrease(MENU_STEP);
            display_edit_value(1);  // 直接显示新数值
        }
        else if(key_down == KEY4)
        {
            // 退出编辑模式
            UI_MenuExitEdit();
            need_redraw = 1;
        }
    }

    // 检测状态变化
    if(menu.state != menu.last_state)
    {
        menu.last_state = menu.state;
        need_redraw = 1;
    }

    // 显示菜单 (只在需要时重绘)
    if(menu.state == UI_MENU_SAVE)
    {
        if(need_redraw) display_menu_save();
    }
    else if(menu.state == UI_MENU_EDIT)
    {
        // 编辑模式：闪烁状态改变或需要重绘时更新
        if(new_blink_state != menu.blink_state)
        {
            menu.blink_state = new_blink_state;
            display_edit_value(menu.blink_state);  // 只更新数值部分
        }
        else if(need_redraw)
        {
            display_menu_edit();  // 完整重绘
        }
    }
    else  // UI_MENU_NORMAL
    {
        if(need_redraw) display_menu_normal();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     进入编辑模式
// 参数说明     void
// 返回参数     void
// 使用示例     UI_MenuEnterEdit();
// 备注信息     从普通模式切换到编辑模式
//-------------------------------------------------------------------------------------------------------------------
void UI_MenuEnterEdit(void)
{
    menu.state = UI_MENU_EDIT;
    menu.item = UI_PID_LEFT_KP;
    menu.value = *get_pid_value_ptr(menu.item);
    menu.blink_cnt = 0;
    menu.blink_state = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     退出编辑模式
// 参数说明     void
// 返回参数     void
// 使用示例     UI_MenuExitEdit();
// 备注信息     从编辑模式切换到普通模式，显示保存提示
//-------------------------------------------------------------------------------------------------------------------
void UI_MenuExitEdit(void)
{
    menu.state = UI_MENU_SAVE;
    menu.save_cnt = MENU_SAVE_DELAY;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     增加当前参数值
// 参数说明     step    步进值
// 返回参数     void
// 使用示例     UI_MenuIncrease(0.01f);
// 备注信息     范围限制: 0.0 ~ 10.0
//-------------------------------------------------------------------------------------------------------------------
void UI_MenuIncrease(float step)
{
    float *ptr = get_pid_value_ptr(menu.item);
    if(ptr != NULL)
    {
        *ptr += step;
        if(*ptr > 10.0f)
        {
            *ptr = 10.0f;
        }
        menu.value = *ptr;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     减少当前参数值
// 参数说明     step    步进值
// 返回参数     void
// 使用示例     UI_MenuDecrease(0.01f);
// 备注信息     范围限制: 0.0 ~ 10.0
//-------------------------------------------------------------------------------------------------------------------
void UI_MenuDecrease(float step)
{
    float *ptr = get_pid_value_ptr(menu.item);
    if(ptr != NULL)
    {
        *ptr -= step;
        if(*ptr < 0.0f)
        {
            *ptr = 0.0f;
        }
        menu.value = *ptr;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     切换到下一个参数项
// 参数说明     void
// 返回参数     void
// 使用示例     UI_MenuNextItem();
// 备注信息     循环切换: L-kp → L-ki → L-kd → R-kp → R-ki → R-kd → L-kp
//-------------------------------------------------------------------------------------------------------------------
void UI_MenuNextItem(void)
{
    ui_pid_item_t next = (ui_pid_item_t)(menu.item + 1);
    if(next >= UI_PID_COUNT)
    {
        menu.item = UI_PID_LEFT_KP;
    }
    else
    {
        menu.item = next;
    }
    menu.value = *get_pid_value_ptr(menu.item);
    menu.blink_cnt = 0;
    menu.blink_state = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取参数名称
// 参数说明     item    参数项
// 返回参数     参数名称字符串
// 使用示例     char *name = UI_MenuGetItemName(UI_PID_LEFT_KP);
// 备注信息     返回如 "L-kp", "R-ki" 等字符串
//-------------------------------------------------------------------------------------------------------------------
char* UI_MenuGetItemName(ui_pid_item_t item)
{
    if(item < UI_PID_COUNT)
    {
        return item_names[item];
    }
    return (char*)"???";
}

/*==================================================================================================================*/
/* =============== 私有函数实现 =============== */
/*==================================================================================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取参数值指针
// 参数说明     item    参数项
// 返回参数     参数值指针
// 备注信息     根据参数项返回对应的 float* 指针
//-------------------------------------------------------------------------------------------------------------------
static float* get_pid_value_ptr(ui_pid_item_t item)
{
    switch(item)
    {
        case UI_PID_LEFT_KP:  return &pid_left.kp;
        case UI_PID_LEFT_KI:  return &pid_left.ki;
        case UI_PID_LEFT_KD:  return &pid_left.kd;
        case UI_PID_RIGHT_KP: return &pid_right.kp;
        case UI_PID_RIGHT_KI: return &pid_right.ki;
        case UI_PID_RIGHT_KD: return &pid_right.kd;
        default: return NULL;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示普通模式界面
// 参数说明     void
// 返回参数     void
// 备注信息     显示菜单标题和提示信息
//-------------------------------------------------------------------------------------------------------------------
static void display_menu_normal(void)
{
    // 清屏
    OLED_Clear();

    // 标题行: "PID MENU     K1=Edit"
    OLED_ShowString(UI_TITLE_LINE, 1, "PID MENU");

    // 第3行: 提示信息
    OLED_ShowString(3, 1, "K1=Edit");


}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示编辑模式界面
// 参数说明     void
// 返回参数     void
// 备注信息     显示当前选中的参数，数值部分闪烁显示
//              第2行: "> L-kp:  1200" (数值闪烁)
//              第4行: "Next: L-ki"
//-------------------------------------------------------------------------------------------------------------------
static void display_menu_edit(void)
{
    int32_t display_val;

    // 清屏
    OLED_Clear();

    // 标题行: "PID EDIT      <>Adj"
    OLED_ShowString(UI_TITLE_LINE, 1, "PID EDIT");
    OLED_ShowString(UI_TITLE_LINE, 10, "<>Adj");

    // 第2行: 当前选中项 "> L-kp:  1200"
    OLED_ShowString(2, 1, ">");
    OLED_ShowString(2, 2, UI_MenuGetItemName(menu.item));
    OLED_ShowString(2, 7, ":");

    // 显示参数值 (x100 显示两位小数)
    display_val = (int32_t)(menu.value * 100);
    if(menu.blink_state)
    {
        OLED_ShowSignedNum(2, 8, display_val, 4);
    }
    else
    {
        OLED_ShowString(2, 8, "    ");  // 闪烁时清空
    }

    // 第4行: 显示下一个参数提示
    if((ui_pid_item_t)(menu.item + 1) < UI_PID_COUNT)
    {
        OLED_ShowString(4, 1, "Next:");
        OLED_ShowString(4, 6, UI_MenuGetItemName((ui_pid_item_t)(menu.item + 1)));
    }
    else
    {
        OLED_ShowString(4, 1, "Next: L-kp");  // 循环回到第一个
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     只更新编辑模式的数值部分 (用于闪烁效果)
// 参数说明     show    是否显示数值 (1=显示, 0=隐藏)
// 返回参数     void
// 备注信息     只刷新数值区域，不清屏
//-------------------------------------------------------------------------------------------------------------------
static void display_edit_value(uint8_t show)
{
    int32_t display_val = (int32_t)(menu.value * 100);

    if(show)
    {
        OLED_ShowSignedNum(2, 8, display_val, 4);
    }
    else
    {
        OLED_ShowString(2, 8, "    ");  // 清空数值
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示保存成功提示
// 参数说明     void
// 返回参数     void
// 备注信息     显示 "SAVED!" 2秒后自动返回普通模式
//-------------------------------------------------------------------------------------------------------------------
static void display_menu_save(void)
{
    OLED_Clear();
    OLED_ShowString(2, 5, "SAVED!");
}
