#include <bsp_config.h>
#include <rtthread.h>
#include <stdbool.h>
#include "gpio.h"
#include "hal_gpio.h"

//中断服务邮箱的容量
#define MAX_IRQ_SIZE    10
#define GPIO_IRQ_SERVICE_STACK_SIZE 512

/* Private variables ---------------------------------------------------------*/
static hal_gpio_irq_handler* irq_arr[4][16];

static struct rt_mailbox mb_irq_service;
static uint8_t mb_irq_serivce_pool[MAX_IRQ_SIZE*4];
static bool is_irq_serivce_initialized = false;
static struct rt_thread t_irq;
static uint8_t irq_serivce_stack[GPIO_IRQ_SERVICE_STACK_SIZE];

/* Public variables ---------------------------------------------------------*/

/* Private prototype ---------------------------------------------------------*/
void Gpio_Exti_IRQHandler(en_gpio_port_t enPort, en_gpio_pin_t enPin);

/* Private Functions ---------------------------------------------------------*/
void PortA_IRQHandler(void)
{   
    Gpio_Exti_IRQHandler(GpioPortA,GpioPin0);
    Gpio_Exti_IRQHandler(GpioPortA,GpioPin1);
    Gpio_Exti_IRQHandler(GpioPortA,GpioPin2);
    Gpio_Exti_IRQHandler(GpioPortA,GpioPin3);
    Gpio_Exti_IRQHandler(GpioPortA,GpioPin4);
    Gpio_Exti_IRQHandler(GpioPortA,GpioPin5);
    Gpio_Exti_IRQHandler(GpioPortA,GpioPin6);
    Gpio_Exti_IRQHandler(GpioPortA,GpioPin7);
    Gpio_Exti_IRQHandler(GpioPortA,GpioPin8);
    Gpio_Exti_IRQHandler(GpioPortA,GpioPin9);
    Gpio_Exti_IRQHandler(GpioPortA,GpioPin10);
    Gpio_Exti_IRQHandler(GpioPortA,GpioPin11);
    Gpio_Exti_IRQHandler(GpioPortA,GpioPin12);
    Gpio_Exti_IRQHandler(GpioPortA,GpioPin13);
    Gpio_Exti_IRQHandler(GpioPortA,GpioPin14);
    Gpio_Exti_IRQHandler(GpioPortA,GpioPin15);
}

void PortB_IRQHandler(void)
{   
    Gpio_Exti_IRQHandler(GpioPortB,GpioPin0);
    Gpio_Exti_IRQHandler(GpioPortB,GpioPin1);
    Gpio_Exti_IRQHandler(GpioPortB,GpioPin2);
    Gpio_Exti_IRQHandler(GpioPortB,GpioPin3);
    Gpio_Exti_IRQHandler(GpioPortB,GpioPin4);
    Gpio_Exti_IRQHandler(GpioPortB,GpioPin5);
    Gpio_Exti_IRQHandler(GpioPortB,GpioPin6);
    Gpio_Exti_IRQHandler(GpioPortB,GpioPin7);
    Gpio_Exti_IRQHandler(GpioPortB,GpioPin8);
    Gpio_Exti_IRQHandler(GpioPortB,GpioPin9);
    Gpio_Exti_IRQHandler(GpioPortB,GpioPin10);
    Gpio_Exti_IRQHandler(GpioPortB,GpioPin11);
    Gpio_Exti_IRQHandler(GpioPortB,GpioPin12);
    Gpio_Exti_IRQHandler(GpioPortB,GpioPin13);
    Gpio_Exti_IRQHandler(GpioPortB,GpioPin14);
    Gpio_Exti_IRQHandler(GpioPortB,GpioPin15);    
}

void PortC_IRQHandler(void)
{ 
    Gpio_Exti_IRQHandler(GpioPortC,GpioPin0);
    Gpio_Exti_IRQHandler(GpioPortC,GpioPin1);
    Gpio_Exti_IRQHandler(GpioPortC,GpioPin2);
    Gpio_Exti_IRQHandler(GpioPortC,GpioPin3);
    Gpio_Exti_IRQHandler(GpioPortC,GpioPin4);
    Gpio_Exti_IRQHandler(GpioPortC,GpioPin5);
    Gpio_Exti_IRQHandler(GpioPortC,GpioPin6);
    Gpio_Exti_IRQHandler(GpioPortC,GpioPin7);
    Gpio_Exti_IRQHandler(GpioPortC,GpioPin8);
    Gpio_Exti_IRQHandler(GpioPortC,GpioPin9);
    Gpio_Exti_IRQHandler(GpioPortC,GpioPin10);
    Gpio_Exti_IRQHandler(GpioPortC,GpioPin11);
    Gpio_Exti_IRQHandler(GpioPortC,GpioPin12);
    Gpio_Exti_IRQHandler(GpioPortC,GpioPin13);
    Gpio_Exti_IRQHandler(GpioPortC,GpioPin14);
    Gpio_Exti_IRQHandler(GpioPortC,GpioPin15);      
}

void PortD_IRQHandler(void)
{   
    Gpio_Exti_IRQHandler(GpioPortD,GpioPin0);
    Gpio_Exti_IRQHandler(GpioPortD,GpioPin1);
    Gpio_Exti_IRQHandler(GpioPortD,GpioPin2);
    Gpio_Exti_IRQHandler(GpioPortD,GpioPin3);
    Gpio_Exti_IRQHandler(GpioPortD,GpioPin4);
    Gpio_Exti_IRQHandler(GpioPortD,GpioPin5);
    Gpio_Exti_IRQHandler(GpioPortD,GpioPin6);
    Gpio_Exti_IRQHandler(GpioPortD,GpioPin7);
    Gpio_Exti_IRQHandler(GpioPortD,GpioPin8);
    Gpio_Exti_IRQHandler(GpioPortD,GpioPin9);
    Gpio_Exti_IRQHandler(GpioPortD,GpioPin10);
    Gpio_Exti_IRQHandler(GpioPortD,GpioPin11);
    Gpio_Exti_IRQHandler(GpioPortD,GpioPin12);
    Gpio_Exti_IRQHandler(GpioPortD,GpioPin13);
    Gpio_Exti_IRQHandler(GpioPortD,GpioPin14);
    Gpio_Exti_IRQHandler(GpioPortD,GpioPin15); 
}

void PortE_IRQHandler(void)
{   
    Gpio_Exti_IRQHandler(GpioPortE,GpioPin0);
    Gpio_Exti_IRQHandler(GpioPortE,GpioPin1);
    Gpio_Exti_IRQHandler(GpioPortE,GpioPin2);
    Gpio_Exti_IRQHandler(GpioPortE,GpioPin3);
    Gpio_Exti_IRQHandler(GpioPortE,GpioPin4);
    Gpio_Exti_IRQHandler(GpioPortE,GpioPin5);
    Gpio_Exti_IRQHandler(GpioPortE,GpioPin6);
    Gpio_Exti_IRQHandler(GpioPortE,GpioPin7);
    Gpio_Exti_IRQHandler(GpioPortE,GpioPin8);
    Gpio_Exti_IRQHandler(GpioPortE,GpioPin9);
    Gpio_Exti_IRQHandler(GpioPortE,GpioPin10);
    Gpio_Exti_IRQHandler(GpioPortE,GpioPin11);
    Gpio_Exti_IRQHandler(GpioPortE,GpioPin12);
    Gpio_Exti_IRQHandler(GpioPortE,GpioPin13);
    Gpio_Exti_IRQHandler(GpioPortE,GpioPin14);
    Gpio_Exti_IRQHandler(GpioPortE,GpioPin15);
}

void PortF_IRQHandler(void)
{
    Gpio_Exti_IRQHandler(GpioPortF,GpioPin0);
    Gpio_Exti_IRQHandler(GpioPortF,GpioPin1);
    Gpio_Exti_IRQHandler(GpioPortF,GpioPin2);
    Gpio_Exti_IRQHandler(GpioPortF,GpioPin3);
    Gpio_Exti_IRQHandler(GpioPortF,GpioPin4);
    Gpio_Exti_IRQHandler(GpioPortF,GpioPin5);
    Gpio_Exti_IRQHandler(GpioPortF,GpioPin6);
    Gpio_Exti_IRQHandler(GpioPortF,GpioPin7);
    Gpio_Exti_IRQHandler(GpioPortF,GpioPin8);
    Gpio_Exti_IRQHandler(GpioPortF,GpioPin9);
    Gpio_Exti_IRQHandler(GpioPortF,GpioPin10);
    Gpio_Exti_IRQHandler(GpioPortF,GpioPin11);
    Gpio_Exti_IRQHandler(GpioPortF,GpioPin12);
    Gpio_Exti_IRQHandler(GpioPortF,GpioPin13);
    Gpio_Exti_IRQHandler(GpioPortF,GpioPin14);
    Gpio_Exti_IRQHandler(GpioPortF,GpioPin15);        
}

void HAL_GPIO_EXTI_Callback(uint8_t enPort, en_gpio_pin_t enPin )
{
    rt_mb_send(&mb_irq_service,enPort<<4|enPin);
    
//    if(irq_arr[enPort][enPin] != NULL)
//        irq_arr[enPort][enPin](NULL);
}

/**
  * @brief  This function handles EXTI interrupt request.
  * @param  GPIO_Pin: Specifies the pins connected to the EXTI line.
  * @retval None
  */
void Gpio_Exti_IRQHandler(en_gpio_port_t enPort, en_gpio_pin_t enPin)
{  
    if(TRUE == Gpio_GetIrqStatus(enPort, enPin))
    {
        Gpio_ClearIrq(enPort, enPin);
        HAL_GPIO_EXTI_Callback(enPort>>6, enPin);        
    }
}

void thread_gpio_irq_service(void* args)
{
    int irq_pin_name = 0;
    while(1)
    {
        if(rt_mb_recv(&mb_irq_service,(rt_uint32_t*)&irq_pin_name,RT_WAITING_FOREVER) == RT_EOK)
        {
            if( irq_arr[irq_pin_name>>4][irq_pin_name&0x0f] != NULL )
            {
                irq_arr[irq_pin_name>>4][irq_pin_name&0x0f](NULL);
            }
        }
    }  
}

/* Public functions ---------------------------------------------------------*/
void hal_gpio_init(hal_gpio_t* gpio,gpio_pin_names pin_name,
                        gpio_pin_modes pin_mode,gpio_pin_configs pin_config,gpio_pin_types pin_type,uint8_t value)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);   

    DDL_ZERO_STRUCT(stcGpioCfg);    

    gpio->pin_name = pin_name;

    if( gpio->pin_name == NC )
    {
        return;
    }

    gpio->pin = (en_gpio_pin_t)(gpio->pin_name & 0x0f);//(en_gpio_pin_t)( 0x01 << ( obj->pin & 0x0F ) );

    if( ( gpio->pin_name & 0xF0 ) == 0x00 )
    {
        gpio->port = GpioPortA;  
    }
    else if( ( gpio->pin_name & 0xF0 ) == 0x10 )
    {
        gpio->port = GpioPortB;
    }
    else if( ( gpio->pin_name & 0xF0 ) == 0x20 )
    {
        gpio->port = GpioPortC;
    }
    else if( ( gpio->pin_name & 0xF0 ) == 0x30 )
    {
        gpio->port = GpioPortD;
    }
    else
    {
//        assert_param( FAIL );
    }
        
    if(pin_type == PIN_PULL_UP)
    {
        stcGpioCfg.enPu = GpioPuEnable;
        stcGpioCfg.enPd = GpioPdDisable;      
    }
    else if(pin_type == PIN_PULL_DOWN)
    {
        stcGpioCfg.enPu = GpioPuDisable;
        stcGpioCfg.enPd = GpioPdEnable;            
    }
    else
    {
        stcGpioCfg.enPu = GpioPuDisable;
        stcGpioCfg.enPd = GpioPdDisable;             
    }


    if( pin_mode == PIN_INPUT )
    {
        stcGpioCfg.enDir = GpioDirIn;
    }
    else if(pin_mode == PIN_OUTPUT) // mode output
    {
        if( pin_config == PIN_OPEN_DRAIN )
        {            
            stcGpioCfg.enOD = GpioOdEnable;
        }
        else
        {
            ///< 端口驱动能力配置->高驱动能力
            stcGpioCfg.enDrv = GpioDrvH;
            stcGpioCfg.enOD = GpioOdDisable;
        }
        
        stcGpioCfg.enDir = GpioDirOut;
    }
    else
    {
        // stcGpioCfg.enDir = GPIO_MODE_ANALOG;

        Gpio_SetAnalogMode((en_gpio_port_t)gpio->port,(en_gpio_pin_t)gpio->pin);
        return;
    }

    Gpio_Init( (en_gpio_port_t)gpio->port, (en_gpio_pin_t)gpio->pin,&stcGpioCfg );

    if(pin_mode == PIN_OUTPUT)
    {
        hal_gpio_write(gpio,value);
    }
}

void hal_gpio_irq_set(hal_gpio_t* gpio,gpio_irq_modes irq_mode,gpio_irq_priority irq_priority,void (*irq_handler)(void*))
{
    en_irq_level_t priority = IrqLevel3;
    IRQn_Type IRQnb = PORTA_IRQn;
    en_gpio_irqtype_t irq_type;

    if(!is_irq_serivce_initialized)
    {
        //初始化中断服务邮箱
        rt_mb_init(&mb_irq_service,"mb_irq",mb_irq_serivce_pool,MAX_IRQ_SIZE,RT_IPC_FLAG_FIFO);        

        //初始化中断服务线程
        rt_thread_init(&t_irq,"t_irq",thread_gpio_irq_service,NULL,irq_serivce_stack,sizeof(irq_serivce_stack),1,10);    
        rt_thread_startup(&t_irq);        

        is_irq_serivce_initialized = true;
    }

    if( irq_handler == NULL )
    {
        return;
    }

    gpio->irq_mode = irq_mode;
    
    if( irq_mode == IRQ_RISING_EDGE )
    {
        irq_type = GpioIrqRising;
    }
    else if( irq_mode == IRQ_FALLING_EDGE )
    {
        irq_type = GpioIrqFalling;
    }
    else if(irq_mode == IRQ_HIGH_LEVEL)
    {
        irq_type = GpioIrqHigh;
    }
    else if(irq_mode == IRQ_LOW_LEVEL)
    {
        irq_type = GpioIrqLow;        
    }   

    switch( irq_priority )
    {
        case IRQ_LOW_PRIORITY:
            priority = IrqLevel3;
            break;
        case IRQ_MEDIUM_PRIORITY:
            priority = IrqLevel2;
            break;
        case IRQ_HIGH_PRIORITY:
            priority = IrqLevel1;
            break;
        case IRQ_VERY_HIGH_PRIORITY:
        default:
            priority = IrqLevel0;
            break;
    }

    switch( gpio->port )
    {
        case GpioPortA:
            IRQnb = PORTA_IRQn;
            break;
        case GpioPortB:
            IRQnb = PORTB_IRQn;      
            break;
        case GpioPortC:
        case GpioPortE:            
            IRQnb = PORTC_E_IRQn;        
            break;
        case GpioPortD:
        case GpioPortF:
            IRQnb = PORTD_F_IRQn;          
            break;
        default:
            break;
    }

    irq_arr[gpio->port>>6][gpio->pin&0x0F] = irq_handler;
    
    Gpio_EnableIrq((en_gpio_port_t)gpio->port, (en_gpio_pin_t)gpio->pin, irq_type);
    EnableNvic(IRQnb, priority, TRUE);
    ///< 深度休眠模式下响应端口中断
    Gpio_SfIrqModeCfg(GpioSfIrqDpslpMode);
}

void hal_gpio_irq_clear( hal_gpio_t* gpio )
{
    en_gpio_irqtype_t irq_type;
    
    if( gpio->irq_mode == IRQ_RISING_EDGE )
    {
        irq_type = GpioIrqRising;
    }
    else if( gpio->irq_mode == IRQ_FALLING_EDGE )
    {
        irq_type = GpioIrqFalling;
    }
    else if(gpio->irq_mode == IRQ_HIGH_LEVEL)
    {
        irq_type = GpioIrqHigh;
    }
    else if(gpio->irq_mode == IRQ_LOW_LEVEL)
    {
        irq_type = GpioIrqLow;        
    }    
    
    Gpio_DisableIrq((en_gpio_port_t)gpio->port,(en_gpio_pin_t)gpio->pin,irq_type);
    irq_arr[gpio->port>>6][gpio->pin&0x0F] = NULL;
}

void hal_gpio_write(hal_gpio_t* gpio,uint8_t value)
{
    if( gpio == NULL )
    {
        return;
    }

    // Check if pin is not connected
    if( gpio->pin == NC )
    {
        return;
    }
    Gpio_WriteOutputIO( (en_gpio_port_t)gpio->port, (en_gpio_pin_t)gpio->pin,value );
}

void hal_gpio_toogle(hal_gpio_t* gpio)
{    
    *((volatile uint32_t *)((uint32_t)&M0P_GPIO->PAOUT + gpio->port)) ^= ((1UL)<<(gpio->pin));
}

uint8_t hal_gpio_read(hal_gpio_t* gpio)
{
    if( gpio == NULL )
    {
        return 255;
    }

    // Check if pin is not connected
    if( gpio->pin == NC )
    {
        return 255;
    }
    return Gpio_GetInputIO( (en_gpio_port_t)gpio->port, (en_gpio_pin_t)gpio->pin );
}

