#include "sht20.h"
#include "hal_gpio.h"
#include "bsp.h"

#define POWER_PINNAME  PB_10

hal_gpio_t sht20_power;                         //电源控制脚


static bool Get_Average(float *buf, int buf_size, float all, int idx , float *avg);
static void bubbleSort(float *arr, int len);
static void delay_us(uint32_t timeout);

void sht20_Init()
{
    //初始化电源脚
    hal_gpio_init(&sht20_power,POWER_PINNAME,PIN_OUTPUT,PIN_PUSH_PULL,PIN_NO_PULL,1); 
}

void sht20_SetPower(bool en)
{
    if(en == POWER_ON)
    {
        hal_gpio_write(&sht20_power,POWER_ON);
        delay_us(1000000);
    }
    else
    {
        hal_gpio_write(&sht20_power,POWER_OFF);
    } 
}

void sht20_SoftReset(i2c_bus_id id)
{
    uint8_t buf[1] = {SOFT_RESET};
    i2c_msg_t msg;
    msg.salve_addr = SHT20_ADDR;
    msg.wr_flag = I2C_WR;
    msg.buf = buf;
    msg.len = 1;
    hal_i2c_transfer(id, &msg, 1);
    
    delay_us(1000000);
}

bool sht20_GetTemprature(i2c_bus_id id, float *temp)
{  
    bool result = false;
    /* 读取温度存放缓存buf */
    uint8_t data[2] = {TRIG_TEMP_MEASUREMENT_HM,0};
    i2c_msg_t msg[2];
    msg[I2C_WR].salve_addr = SHT20_ADDR;
    msg[I2C_WR].wr_flag = I2C_WR;
    msg[I2C_WR].buf = data;
    msg[I2C_WR].len = 1;
    msg[I2C_RD].salve_addr = SHT20_ADDR;
    msg[I2C_RD].wr_flag = I2C_RD;
    msg[I2C_RD].buf = data;
    msg[I2C_RD].len = 2;
    
    if(hal_i2c_transfer(id, msg, 2) == 0)
    {
        result = true;
    }
    else
    {
        result = false;
    }

    /* 计算温度 */
    *temp = (float)(((data[0]<<8) | data[1]) & (~0x0003)) * 0.00268127 - 46.85;
    
    return result;
}

bool sht20_GetHumility(i2c_bus_id id,float *humi)
{   
    bool result = false; 

    /* 读取湿度存放缓存buf */
    uint8_t data[2] = {TRIG_HUMI_MEASUREMENT_HM,0};
    i2c_msg_t msg[2];
    msg[I2C_WR].salve_addr = SHT20_ADDR;
    msg[I2C_WR].wr_flag = I2C_WR;
    msg[I2C_WR].buf = data;
    msg[I2C_WR].len = 1;
    msg[I2C_RD].salve_addr = SHT20_ADDR;
    msg[I2C_RD].wr_flag = I2C_RD;
    msg[I2C_RD].buf = data;
    msg[I2C_RD].len = 2;
    
    if(hal_i2c_transfer(id, msg, 2) == 0)
    {
        result = true;
    }
    else
    {
        result = false;
    }

    /* 计算温度 */
    *humi = (float)(((data[0]<<8) | data[1]) & (~0x0003)) * 0.00190735 - 6;    
    
    return result;
}

bool sht20_GetTempAndHumi_Average(i2c_bus_id id, float *temp, float *humi)
{
    sht20_SetPower(POWER_ON);
    sht20_SoftReset(id);
    
    float temp_buf[5] = {0};
    float humi_buf[5] = {0};
    int i = 0;
    int t_idx = 0;
    int h_idx = 0;
    float t = 0, t_all = 0;
    float h = 0, h_all = 0;
    
    //获取正常数据,并加起来
	for(i=0;i<5;i++)
    {          
        sht20_GetHumility(id, &h);
		sht20_GetTemprature(id, &t);
        
        rt_kprintf("temp = %d\r\nhumi = %d\n", (int16_t)(t*100), (uint16_t)(h*100));
        
        if(t<=120 && t>=-40)
        {
            temp_buf[i] = t;
            t_idx++;//正常值计数
		}
        
        if(h<=120 && h>=0)
        {
            humi_buf[i] = h;
            h_idx++;
        }
       
        t_all += temp_buf[i];
        h_all += humi_buf[i];
        
        delay_us(30000);
    }
    
    sht20_SetPower(POWER_OFF);
    
    /* 无正常值 */
    if(t_idx == 0 || h_idx == 0)
    {
        return false;
    }
    
    Get_Average(temp_buf, 5, t_all, t_idx, temp);
    Get_Average(humi_buf, 5, h_all, h_idx, humi);

    return true;
}

static bool Get_Average(float *buf, int buf_size, float all, int idx , float *avg)
{
    if(idx == 0 || buf_size == 0 || all==0)
        return false;
    
    int j = 0;
    
    if(idx>2)
    {
        bubbleSort(buf, buf_size);
        
        float H_temp = buf[0];
        float L_temp = buf[0];      
        for(j=0; j<idx; j++)
        {        
            if(H_temp < buf[j])
            {
                H_temp = buf[j];
            }
            if(L_temp > buf[j])
            {
                L_temp = buf[j];
            }
        }
        //去最高最低
        all = all - H_temp - L_temp;
        all = all/(idx-2);
    }
    else if(idx == 2)
    {
        all = all/2;
    } 
    
    *avg  = all;
    
    return true;
}

/* 冒泡排序 */
/* 1. 从当前元素起，向后依次比较每一对相邻元素，若逆序则交换 */
/* 2. 对所有元素均重复以上步骤，直至最后一个元素 */
/* elemType arr[]: 排序目标数组; int len: 元素个数 */
static void bubbleSort (float *arr, int len) 
{
    float temp;
    int i, j;

    /* 外循环为排序趟数，len个数进行len-1趟 */
    for (i=0; i<len-1; i++) 
    {
        /* 内循环为每趟比较的次数，第i趟比较len-i次 */
        for (j=0; j<len-1-i; j++) 
        { 
            /* 相邻元素比较，若逆序则交换（升序为左大于右，降序反之） */
            if (arr[j] < arr[j+1]) 
            {              
                temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = temp;
            }
        }
    }
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
