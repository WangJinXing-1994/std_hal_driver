/*
 * @Author chenjk
 * @Date 2020-04-07 10:00:25
 * @LastEditTime 2020-06-01 11:20:30
 * @FilePath \rtt-hc32-sdk\hal\inc\hal_gpio.h
 * @Description 
 *      1. 配置GPIO口的输入输出相关(输入/输出，上下拉，推挽/开漏，输出状态初值
 *      2. 配置GPIO的中断相关(触发方式，中断优先级，中断回调函数)
 */ 

#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @addtogroup HAL_GPIO
 * @brief HAL GPIO API
 * @{
 */

#include <stdint.h>

#define MCU_PINS \
    PA_0 = 0, PA_1, PA_2, PA_3, PA_4, PA_5, PA_6, PA_7, PA_8, PA_9, PA_10, PA_11, PA_12, PA_13, PA_14, PA_15, \
    PB_0, PB_1, PB_2, PB_3, PB_4, PB_5, PB_6, PB_7, PB_8, PB_9, PB_10, PB_11, PB_12, PB_13, PB_14, PB_15,     \
    PC_0, PC_1, PC_2, PC_3, PC_4, PC_5, PC_6, PC_7, PC_8, PC_9, PC_10, PC_11, PC_12, PC_13, PC_14, PC_15,     \
    PD_0, PD_1, PD_2, PD_3, PD_4, PD_5, PD_6, PD_7, PD_8, PD_9, PD_10, PD_11, PD_12, PD_13, PD_14, PD_15,     \
    PE_0, PE_1, PE_2, PE_3, PE_4, PE_5, PE_6, PE_7, PE_8, PE_9, PE_10, PE_11, PE_12, PE_13, PE_14, PE_15,     \
    PF_0, PF_1, PF_2, PF_3, PF_4, PF_5, PF_6, PF_7, PF_8, PF_9, PF_10, PF_11, PF_12, PF_13, PF_14, PF_15,     \
    PH_0, PH_1, PH_2, PH_3, PH_4, PH_5, PH_6, PH_7, PH_8, PH_9, PH_10, PH_11, PH_12, PH_13, PH_14, PH_15

/**
 * @brief 引脚名称定义
 */
typedef enum
{
    MCU_PINS,

    NC=0xff
} gpio_pin_names;

/**
 * @brief GPIO 引脚输入输出模式
 */
typedef enum
{
    PIN_INPUT = 0,
    PIN_OUTPUT,
    PIN_ANALOGIC
} gpio_pin_modes;

/**
 * @brief GPIO 引脚上下拉
 */
typedef enum
{
    PIN_NO_PULL = 0,
    PIN_PULL_UP,
    PIN_PULL_DOWN
}gpio_pin_types;

/**
 * @brief GPIO输出方式定义
 */
typedef enum
{
    PIN_PUSH_PULL = 0,
    PIN_OPEN_DRAIN
} gpio_pin_configs;

/**
 * @brief GPIO中断触发方式定义
 */
typedef enum
{
    NO_IRQ = 0,
    IRQ_RISING_EDGE,
    IRQ_FALLING_EDGE,
    IRQ_HIGH_LEVEL,
    IRQ_LOW_LEVEL
} gpio_irq_modes;

/**
 * @brief GPIO中断优先级触发
 */
typedef enum
{
    IRQ_LOW_PRIORITY = 0,
    IRQ_MEDIUM_PRIORITY,
    IRQ_HIGH_PRIORITY,
    IRQ_VERY_HIGH_PRIORITY
} gpio_irq_priority;

/**
 * @brief GPIO hal 引脚控制块
 */
typedef struct
{
    gpio_pin_names pin_name;
    uint32_t port;
    uint8_t pin;
    gpio_irq_modes irq_mode;
} hal_gpio_t;

/**
 * @brief GPIO中断回调函数定义
 */
typedef void (hal_gpio_irq_handler)(void*);

/**
 * @brief 根据pin_name和设置信息来配置IO口，并将配置信息保存在gpio的hal层引脚控制块中
 * @param gpio gpio外设控制块
 * @param pin_name 引脚名称
 * @param pin_mode 引脚输入输出模式
 * @param pin_config 引脚输出方式(输出模式时有效)
 * @param pin_type 引脚上下拉
 * @param value 引脚输出电平(输出模式时有效)
 */
void hal_gpio_init(hal_gpio_t* gpio,gpio_pin_names pin_name,
                        gpio_pin_modes pin_mode,gpio_pin_configs pin_config,gpio_pin_types pin_type,uint8_t value);

/**
 * @brief gpio中断设置
 * @param gpio          gpio外设控制块 
 * @param irq_mode      中断触发方式
 * @param irq_priority  中断优先级
 * @param irq_handler   中断回调函数
 */
void hal_gpio_irq_set(hal_gpio_t* gpio,gpio_irq_modes irq_mode,gpio_irq_priority irq_priority,void (*irq_handler)(void*));

/**
 * @brief 清除中断
 * @param gpio gpio外设控制块 
 */
void hal_gpio_irq_clear(hal_gpio_t* gpio);

/**
 * @brief gpio写
 * @param gpio      gpio外设控制块
 * @param value     引脚电平(0=>低电平，1=>高电平) 
 */
void hal_gpio_write(hal_gpio_t* gpio,uint8_t value);

/**
 * @brief gpio读
 * @param gpio      gpio外设控制块
 * @retval 0=> 低电平
 * @retval 1=> 高电平 
 */
uint8_t hal_gpio_read(hal_gpio_t* gpio);

/**
 * @brief gpio电平转换
 * @param gpio      gpio外设控制块
 */
void hal_gpio_toogle(hal_gpio_t* gpio);



/**
 * @} 
 */

#ifdef __cplusplus
}
#endif

#endif

