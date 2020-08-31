#include "hal_i2c.h"
#include "hal_gpio.h"
#include "bsp.h"

#define I2C_BUS_MAX     4                       //最大通道数
#define USE_GPIO_PULL_UP                        //无外部上拉电阻需内部上拉   
#define USE_GPIO_CFG_OD                         

#define PIN_HIGH    1                           //高电平
#define PIN_LOW     0                           //低电平

#define I2C0_SCL_PINNAME        PB_8
#define I2C0_SDA_PINNAME        PB_9
#define I2C1_SCL_PINNAME        PA_11
#define I2C1_SDA_PINNAME        PA_12
#define I2C2_SCL_PINNAME        PB_6
#define I2C2_SDA_PINNAME        PB_7
#define I2C3_SCL_PINNAME        PB_10
#define I2C3_SDA_PINNAME        PB_11

#define DELAY(num)              delay_us(num)   //延时函数          
#define ONE_MS_TICK             2000            //2ms = 2000us

typedef enum {
    ACK                         = 0,
    NO_ACK                      = 1,
    ACK_OK                      = 0x00,
    ACK_ERROR                   = 0x01
}i2c_ack;

typedef struct {
    hal_gpio_t *gpio;
    gpio_pin_names name;
    gpio_pin_modes mode;
    gpio_pin_configs cfg;
    gpio_pin_types type;
    uint8_t value;
}i2c_init_t;

typedef struct{
    i2c_init_t scl;
    i2c_init_t sda;
    bool transferring;                  //是否传输数据中
}i2c_t;

i2c_t i2c[I2C_BUS_MAX];

hal_gpio_t i2c0_scl;
hal_gpio_t i2c0_sda;
hal_gpio_t i2c1_scl;
hal_gpio_t i2c1_sda;
hal_gpio_t i2c2_scl;
hal_gpio_t i2c2_sda;
hal_gpio_t i2c3_scl;
hal_gpio_t i2c3_sda;

static void delay_us(uint32_t timeout);
static void i2c_SDA_SetPin(i2c_t *i2c, uint8_t state);
static void i2c_SCL_SetPin(i2c_t *i2c, uint8_t state);
static uint8_t i2c_SDA_GetPin(i2c_t *i2c);
static uint8_t i2c_SCL_GetPin(i2c_t *i2c);
static void i2c_SCL_SetMode(i2c_t *i2c, gpio_pin_modes mode, uint8_t value);
static void i2c_SDA_SetMode(i2c_t *i2c, gpio_pin_modes mode, uint8_t value);

static void i2c_StartCondition(i2c_t *i2c);
static void i2c_StopCondition(i2c_t *i2c);
static void i2c_Acknowledge(i2c_t *i2c);
static void i2c_NoAcknowledge(i2c_t *i2c);
static uint8_t i2c_ReadByte(i2c_t *i2c);
static uint8_t i2c_WriteByte(i2c_t *i2c,uint8_t byte);


void hal_i2c_init(i2c_bus_id id)
{  
    /* 不同通道注册不同i2c命名 */
    switch((int)id)
    {
        case I2C_BUS_0:
        {
            i2c[id].scl.gpio = &i2c0_scl;
            i2c[id].scl.name = I2C0_SCL_PINNAME;
            i2c[id].sda.gpio = &i2c0_sda;
            i2c[id].sda.name = I2C0_SDA_PINNAME;          
        }
        break;
        
        case I2C_BUS_1:
        {
            i2c[id].scl.gpio = &i2c1_scl;
            i2c[id].scl.name = I2C1_SCL_PINNAME;
            i2c[id].sda.gpio = &i2c1_sda;
            i2c[id].sda.name = I2C1_SDA_PINNAME;  
        }
        break;
        
        case I2C_BUS_2:
        {
            i2c[id].scl.gpio = &i2c2_scl;
            i2c[id].scl.name = I2C2_SCL_PINNAME;
            i2c[id].sda.gpio = &i2c2_sda;
            i2c[id].sda.name = I2C2_SDA_PINNAME;  
        }
        break;
        
        case I2C_BUS_3:
        {
            i2c[id].scl.gpio = &i2c3_scl;
            i2c[id].scl.name = I2C3_SCL_PINNAME;
            i2c[id].sda.gpio = &i2c3_sda;
            i2c[id].sda.name = I2C3_SDA_PINNAME;  
        }
        break;
    }  
    
    /* 默认参数 */
    i2c[id].scl.mode = PIN_OUTPUT;
    i2c[id].scl.value = 1;
    i2c[id].sda.mode = PIN_OUTPUT;
    i2c[id].sda.value = 1;
    
    #ifdef USE_GPIO_CFG_OD
    i2c[id].scl.cfg = PIN_OPEN_DRAIN;
    i2c[id].sda.cfg = PIN_OPEN_DRAIN;
    #else
    i2c[id].scl.cfg = PIN_PUSH_PULL;
    i2c[id].sda.cfg = PIN_PUSH_PULL;
    #endif
    
    #ifdef USE_GPIO_PULL_UP
    i2c[id].scl.type = PIN_PULL_UP;
    i2c[id].sda.type = PIN_PULL_UP;
    #else
    i2c[id].scl.type = PIN_NO_PULL;
    i2c[id].sda.type = PIN_NO_PULL;
    #endif  
    
    i2c[id].transferring = false;
    
    /* SCL引脚初始化 */
    hal_gpio_init(i2c[id].scl.gpio, 
                  i2c[id].scl.name, 
                  i2c[id].scl.mode,
                  i2c[id].scl.cfg,
                  i2c[id].scl.type,
                  i2c[id].scl.value);
    /* SDA引脚初始化 */
    hal_gpio_init(i2c[id].sda.gpio, 
                  i2c[id].sda.name, 
                  i2c[id].sda.mode,
                  i2c[id].sda.cfg,
                  i2c[id].sda.type,
                  i2c[id].sda.value);
    DELAY(5);
}

void hal_i2c_deinit(i2c_bus_id id)
{
        /* 不同通道注册不同i2c命名 */
    switch((int)id)
    {
        case I2C_BUS_0:
        {
            i2c[id].scl.gpio = &i2c0_scl;
            i2c[id].scl.name = I2C0_SCL_PINNAME;
            i2c[id].sda.gpio = &i2c0_sda;
            i2c[id].sda.name = I2C0_SDA_PINNAME;          
        }
        break;
        
        case I2C_BUS_1:
        {
            i2c[id].scl.gpio = &i2c1_scl;
            i2c[id].scl.name = I2C1_SCL_PINNAME;
            i2c[id].sda.gpio = &i2c1_sda;
            i2c[id].sda.name = I2C1_SDA_PINNAME;  
        }
        break;
        
        case I2C_BUS_2:
        {
            i2c[id].scl.gpio = &i2c2_scl;
            i2c[id].scl.name = I2C2_SCL_PINNAME;
            i2c[id].sda.gpio = &i2c2_sda;
            i2c[id].sda.name = I2C2_SDA_PINNAME;  
        }
        break;
        
        case I2C_BUS_3:
        {
            i2c[id].scl.gpio = &i2c3_scl;
            i2c[id].scl.name = I2C3_SCL_PINNAME;
            i2c[id].sda.gpio = &i2c3_sda;
            i2c[id].sda.name = I2C3_SDA_PINNAME;  
        }
        break;
    }  
    
    /* 默认参数 */
    i2c[id].scl.mode = PIN_OUTPUT;
    i2c[id].scl.value = 0;
    i2c[id].sda.mode = PIN_OUTPUT;
    i2c[id].sda.value = 0;
    
    #ifdef USE_GPIO_CFG_OD
    i2c[id].scl.cfg = PIN_OPEN_DRAIN;
    i2c[id].sda.cfg = PIN_OPEN_DRAIN;
    #else
    i2c[id].scl.cfg = PIN_PUSH_PULL;
    i2c[id].sda.cfg = PIN_PUSH_PULL;
    #endif

    i2c[id].scl.type = PIN_NO_PULL;
    i2c[id].sda.type = PIN_NO_PULL;
    
    i2c[id].transferring = false;
    
    /* SCL引脚初始化 */
    hal_gpio_init(i2c[id].scl.gpio, 
                  i2c[id].scl.name, 
                  i2c[id].scl.mode,
                  i2c[id].scl.cfg,
                  i2c[id].scl.type,
                  i2c[id].scl.value);
    /* SDA引脚初始化 */
    hal_gpio_init(i2c[id].sda.gpio, 
                  i2c[id].sda.name, 
                  i2c[id].sda.mode,
                  i2c[id].sda.cfg,
                  i2c[id].sda.type,
                  i2c[id].sda.value);   
}

int hal_i2c_transfer(i2c_bus_id id,i2c_msg_t i2c_msg[],uint32_t num)
{ 
    int bus_id = (int)id;

    if(bus_id>=I2C_BUS_MAX || i2c[id].transferring)
        return -1;
    
    /* 关闭线程调度 */
    rt_enter_critical();
    
    /* 该通道正在传输 */
    i2c[id].transferring = true;
    
    int i = 0;
    int j = 0;
    int retry = 100;
    
    /* msg元素都需处理 */
    for(i=0; i<num; i++)
    {
        i2c_StartCondition(&i2c[bus_id]);
        
        /* 写入从机地址 7bit地址 + 1bit读写标志*/
        i2c_WriteByte(&i2c[bus_id], 
                     (i2c_msg[i].salve_addr<<1) + i2c_msg[i].wr_flag);
        
        /* 发送 */
        if(i2c_msg[i].wr_flag == I2C_WR)
        {
            /* 每个字节都连续发送 */
            for(j=0; j<i2c_msg[i].len; j++)
            {
                i2c_WriteByte(&i2c[bus_id], i2c_msg[i].buf[j]);
            }                  
        }
        /* 读取 */
        else if(i2c_msg[i].wr_flag == I2C_RD)
        {                                    
            i2c_SCL_SetPin(&i2c[bus_id], PIN_HIGH);
            i2c_SCL_SetMode(&i2c[bus_id], PIN_INPUT, 0);
            
            /* 超时检查 2ms*/           
            while(i2c_SCL_GetPin(&i2c[bus_id]) == PIN_LOW  && retry>0)
            {
                DELAY(ONE_MS_TICK);
                retry--;
            }
            /* 异常退出 */
            if(retry == 0)
            {
                i2c[id].transferring = false;
                /* 开启线程调度 */
                rt_exit_critical();
                return -1;
            }
            
            /* 每个字节都读取完 */
            for(j=0; j<i2c_msg[i].len; j++)
            {
                i2c_msg[i].buf[j] = i2c_ReadByte(&i2c[bus_id]);
                
                /* 未读取完的ACK */
                if(j<(i2c_msg[i].len-1))
                    i2c_Acknowledge(&i2c[bus_id]);
            }

            i2c_NoAcknowledge(&i2c[bus_id]);
            
            i2c_SCL_SetMode(&i2c[bus_id], PIN_OUTPUT, 0);
        }               
    }
    
    i2c_StopCondition(&i2c[bus_id]);
    
    /* 传输结束 */
    i2c[id].transferring = false;
    
    /* 开启线程调度 */
    rt_exit_critical();
    
    return 0;  
}

//Delay us级 超时,支持大于1ms
static void delay_us(uint32_t timeout)
{
    uint32_t last_tick = SysTick->VAL; //获取当前系统tick
    
    //SYSTICK 是一个递减的定时器
    uint32_t tickNum = 0;
    uint32_t reload_tick = SysTick->LOAD;

    uint32_t current_tick = 0;
    
    uint32_t delta_tick = 0;

    uint32_t timeout_tick = 32 * timeout; //计算一共需要走多少tick

    tickNum = 0;
    

    while (tickNum <= timeout_tick)
    {
        current_tick = SysTick->VAL; //获取当前时间

        if (current_tick > last_tick) //重载了
        {
            delta_tick = reload_tick + last_tick - current_tick;
        }
        else //没有重载
        {
            //因为这是一个递减的寄存器，所以用last_tick减去当前计时器的current_tick表示走过的tick数量
            delta_tick = last_tick - current_tick;
        }

        tickNum += delta_tick;

        last_tick = current_tick;
    }
}

static void i2c_SDA_SetPin(i2c_t *i2c, uint8_t state)
{
    i2c->sda.value = state;
    hal_gpio_write(i2c->sda.gpio, state);
}
static void i2c_SCL_SetPin(i2c_t *i2c, uint8_t state)
{
    i2c->scl.value = state;
    hal_gpio_write(i2c->scl.gpio, state);
}

static uint8_t i2c_SDA_GetPin(i2c_t *i2c)
{
    i2c->sda.value = hal_gpio_read(i2c->sda.gpio);
    return i2c->sda.value;
}
static uint8_t i2c_SCL_GetPin(i2c_t *i2c)
{
    i2c->scl.value = hal_gpio_read(i2c->scl.gpio);
    return i2c->scl.value;
}


//value只对输出有效
static void i2c_SCL_SetMode(i2c_t *i2c, gpio_pin_modes mode, uint8_t value)
{
    if(mode == PIN_INPUT)
    {
        i2c->scl.mode = PIN_INPUT;
    }
    else if(mode == PIN_OUTPUT)
    {
        i2c->scl.mode = PIN_OUTPUT;
        i2c->scl.cfg = PIN_OPEN_DRAIN;
        i2c->scl.value = value;
    }
    
    hal_gpio_init(i2c->scl.gpio, i2c->scl.name, i2c->scl.mode, i2c->scl.cfg, i2c->scl.type, i2c->scl.value);
}
static void i2c_SDA_SetMode(i2c_t *i2c, gpio_pin_modes mode , uint8_t value)
{
    if(mode == PIN_INPUT)
    {
        i2c->sda.mode = PIN_INPUT;
    }
    else if(mode == PIN_OUTPUT)
    {
        i2c->sda.mode = PIN_OUTPUT;
        i2c->sda.cfg = PIN_OPEN_DRAIN;
        i2c->sda.value = value;
    }
    
    hal_gpio_init(i2c->sda.gpio, i2c->sda.name, i2c->sda.mode, i2c->sda.cfg, i2c->sda.type, i2c->sda.value);
}

static void i2c_StartCondition(i2c_t *i2c)
{
    i2c_SCL_SetMode(i2c, PIN_OUTPUT, PIN_HIGH);
    i2c_SDA_SetMode(i2c, PIN_OUTPUT, PIN_HIGH);   
    DELAY(5); 
    i2c_SDA_SetPin(i2c, PIN_LOW);  
    DELAY(5);    
    i2c_SCL_SetPin(i2c, PIN_LOW);
    DELAY(5);
}
static void i2c_StopCondition(i2c_t *i2c)
{  
    i2c_SDA_SetMode(i2c, PIN_OUTPUT, PIN_LOW);
    i2c_SCL_SetMode(i2c, PIN_OUTPUT, PIN_HIGH);
    DELAY(5);    
    i2c_SDA_SetPin(i2c, PIN_HIGH);
    DELAY(5);
}

static void i2c_Acknowledge(i2c_t *i2c)
{    
    i2c_SDA_SetMode(i2c, PIN_OUTPUT, PIN_LOW);
    i2c_SCL_SetMode(i2c, PIN_OUTPUT, PIN_HIGH);
    DELAY(5);
    i2c_SCL_SetPin(i2c, PIN_LOW);
    DELAY(5);
}

static void i2c_NoAcknowledge(i2c_t *i2c)
{
    i2c_SDA_SetMode(i2c, PIN_OUTPUT, PIN_HIGH);
    i2c_SCL_SetMode(i2c, PIN_OUTPUT, PIN_HIGH);
    DELAY(5);
    i2c_SCL_SetPin(i2c, PIN_LOW);
    DELAY(5);
}

static uint8_t i2c_ReadByte(i2c_t *i2c)
{
    int i = 0;
    uint8_t val = 0;
    
    i2c_SDA_SetMode(i2c, PIN_OUTPUT, PIN_HIGH);
    i2c_SDA_SetMode(i2c, PIN_INPUT, 0);   
    
    for(i = 0; i < 8; i++)
    {
        val <<= 1;
        
        i2c_SCL_SetMode(i2c, PIN_OUTPUT, PIN_HIGH);        
        DELAY(5);
            
        if(i2c_SDA_GetPin(i2c) == PIN_HIGH)
        {
            val |= 0x01;
        }
             
        i2c_SCL_SetPin(i2c, PIN_LOW);
        DELAY(5);
    }
    
    i2c_SDA_SetMode(i2c, PIN_OUTPUT, PIN_HIGH);
    
    return val;
}

static uint8_t i2c_WriteByte(i2c_t *i2c,uint8_t byte)
{
    int i = 0;
    uint8_t ack = 0;

    for(i = 0; i < 8; i++)
    {
        if(byte & 0x80) 
        {
            i2c_SDA_SetMode(i2c, PIN_OUTPUT, PIN_HIGH);
        }
        else
        {
            i2c_SDA_SetMode(i2c, PIN_OUTPUT, PIN_LOW);
        }
        
        i2c_SCL_SetMode(i2c, PIN_OUTPUT, PIN_HIGH);
        DELAY(5);
        i2c_SCL_SetPin(i2c, PIN_LOW);
        DELAY(5);
        
        byte <<= 1;
    }
      
    i2c_SDA_SetMode(i2c, PIN_INPUT, 0);
    
    i2c_SCL_SetPin(i2c, PIN_HIGH);
    DELAY(5);
     
    if(i2c_SDA_GetPin(i2c) == PIN_HIGH)
    {
        ack = ACK_ERROR;
    }
    else
    {
        ack = ACK_OK;
    }

    i2c_SCL_SetPin(i2c, PIN_LOW);
    DELAY(5);
    
    i2c_SDA_SetMode(i2c, PIN_OUTPUT, PIN_HIGH);
    
    return ack;
}


