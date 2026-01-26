/*********************************************************************************************************************
 * @file        mygpio.c
 * @brief       GPIO驱动实现 - ZF API风格 (适配STM32F103)
 * @platform    STM32F103C8T6
********************************************************************************************************************/

#include "mygpio.h"

/*==================================================================================================================*/
/* =============== 内部辅助函数 =============== */
/*==================================================================================================================*/

/**
 * @brief       从引脚枚举提取端口
 */
static inline GPIO_TypeDef* gpio_get_port(gpio_pin_enum pin)
{
    uint8_t port_index = (pin >> 8) & 0xFF;

    switch (port_index)
    {
        case 0: return GPIOA;
        case 1: return GPIOB;
        case 2: return GPIOC;
        case 3: return GPIOD;
        default: return NULL;
    }
}

/**
 * @brief       从引脚枚举提取引脚号
 */
static inline uint16_t gpio_get_pin(gpio_pin_enum pin)
{
    return (uint16_t)(pin & 0xFFFF);
}

/**
 * @brief       使能GPIO端口时钟
 */
static void gpio_enable_clock(GPIO_TypeDef *port)
{
    if (port == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (port == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (port == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    else if (port == GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
}

/**
 * @brief       获取GPIO模式 (STM32库格式)
 */
static inline GPIOMode_TypeDef gpio_get_stm_mode(gpio_dir_enum dir, gpio_mode_enum mode)
{
    if (dir == GPI)
    {
        // 输入模式
        if (mode == GPI_PULL_UP) return GPIO_Mode_IPU;
        if (mode == GPI_PULL_DOWN) return GPIO_Mode_IPD;
        return GPIO_Mode_IN_FLOATING;
    }
    else if (dir == GPO || dir == GPIO)
    {
        // 输出模式
        if (mode == GPO_OPEN_DRAIN) return GPIO_Mode_Out_OD;
        return GPIO_Mode_Out_PP;
    }
    return GPIO_Mode_Out_PP;
}

/**
 * @brief       获取GPIO速度 (STM32库格式)
 */
static inline GPIOSpeed_TypeDef gpio_get_stm_speed(gpio_speed_enum speed)
{
    if (speed == GPIO_SPEED_HIGH) return GPIO_Speed_50MHz;
    if (speed == GPIO_SPEED_MED) return GPIO_Speed_2MHz;
    return GPIO_Speed_10MHz;
}

/*==================================================================================================================*/
/* =============== API实现 =============== */
/*==================================================================================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPIO初始化
// 参数说明     pin         引脚选择
// 参数说明     dir         引脚方向: GPI=输入, GPO=输出, GPIO=双向
// 参数说明     dat         初始电平状态 (仅输出有效)
// 参数说明     mode        GPIO模式
// 返回参数     void
// 使用示例     gpio_init(PA0, GPO, 1, GPO_PUSH_PULL);  // PA0推挽输出，初始高电平
// 备注信息     自动使能对应端口时钟
//-------------------------------------------------------------------------------------------------------------------
void gpio_init(gpio_pin_enum pin, gpio_dir_enum dir, const uint8_t dat, gpio_mode_enum mode)
{
    GPIO_TypeDef *port = gpio_get_port(pin);
    uint16_t gpio_pin = gpio_get_pin(pin);
    GPIO_InitTypeDef gpio_cfg = {0};

    if (port == NULL) return;

    // 使能时钟
    gpio_enable_clock(port);

    // 配置GPIO
    gpio_cfg.GPIO_Pin = gpio_pin;
    gpio_cfg.GPIO_Mode = gpio_get_stm_mode(dir, mode);
    gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &gpio_cfg);

    // 设置初始电平 (仅输出模式)
    if (dir == GPO || dir == GPIO)
    {
        if (dat)
            port->BSRR = gpio_pin;   // 置高
        else
            port->BRR = gpio_pin;    // 置低
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPIO电平设置
// 参数说明     pin         引脚选择
// 参数说明     dat         0=低电平, 1=高电平
// 返回参数     void
// 使用示例     gpio_set_level(PA0, 1);  // PA0置高
//-------------------------------------------------------------------------------------------------------------------
void gpio_set_level(gpio_pin_enum pin, uint8_t dat)
{
    GPIO_TypeDef *port = gpio_get_port(pin);
    uint16_t gpio_pin = gpio_get_pin(pin);

    if (port == NULL) return;

    if (dat)
        port->BSRR = gpio_pin;   // 置高
    else
        port->BRR = gpio_pin;    // 置低
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPIO电平读取
// 参数说明     pin         引脚选择
// 返回参数     uint8       0=低电平, 1=高电平
// 使用示例     uint8 level = gpio_get_level(PA0);
//-------------------------------------------------------------------------------------------------------------------
uint8_t gpio_get_level(gpio_pin_enum pin)
{
    GPIO_TypeDef *port = gpio_get_port(pin);
    uint16_t gpio_pin = gpio_get_pin(pin);

    if (port == NULL) return 0;

    return (port->IDR & gpio_pin) ? 1 : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPIO电平翻转
// 参数说明     pin         引脚选择
// 返回参数     void
// 使用示例     gpio_toggle_level(PA0);  // 翻转PA0
//-------------------------------------------------------------------------------------------------------------------
void gpio_toggle_level(gpio_pin_enum pin)
{
    gpio_set_level(pin, !gpio_get_level(pin));
}
