#include "sht20.h"
#include "hal_gpio.h"
#include "bsp.h"

#define POWER_PINNAME  PB_10

hal_gpio_t sht20_power;                         //��Դ���ƽ�


static bool Get_Average(float *buf, int buf_size, float all, int idx , float *avg);
static void bubbleSort(float *arr, int len);
static void delay_us(uint32_t timeout);

void sht20_Init()
{
    //��ʼ����Դ��
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
    /* ��ȡ�¶ȴ�Ż���buf */
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

    /* �����¶� */
    *temp = (float)(((data[0]<<8) | data[1]) & (~0x0003)) * 0.00268127 - 46.85;
    
    return result;
}

bool sht20_GetHumility(i2c_bus_id id,float *humi)
{   
    bool result = false; 

    /* ��ȡʪ�ȴ�Ż���buf */
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

    /* �����¶� */
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
    
    //��ȡ��������,��������
	for(i=0;i<5;i++)
    {          
        sht20_GetHumility(id, &h);
		sht20_GetTemprature(id, &t);
        
        rt_kprintf("temp = %d\r\nhumi = %d\n", (int16_t)(t*100), (uint16_t)(h*100));
        
        if(t<=120 && t>=-40)
        {
            temp_buf[i] = t;
            t_idx++;//����ֵ����
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
    
    /* ������ֵ */
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
        //ȥ������
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

/* ð������ */
/* 1. �ӵ�ǰԪ����������αȽ�ÿһ������Ԫ�أ��������򽻻� */
/* 2. ������Ԫ�ؾ��ظ����ϲ��裬ֱ�����һ��Ԫ�� */
/* elemType arr[]: ����Ŀ������; int len: Ԫ�ظ��� */
static void bubbleSort (float *arr, int len) 
{
    float temp;
    int i, j;

    /* ��ѭ��Ϊ����������len��������len-1�� */
    for (i=0; i<len-1; i++) 
    {
        /* ��ѭ��Ϊÿ�˱ȽϵĴ�������i�˱Ƚ�len-i�� */
        for (j=0; j<len-1-i; j++) 
        { 
            /* ����Ԫ�رȽϣ��������򽻻�������Ϊ������ң�����֮�� */
            if (arr[j] < arr[j+1]) 
            {              
                temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = temp;
            }
        }
    }
}

//Delay us�� ��ʱ,֧�ִ���1ms
static void delay_us(uint32_t timeout)
{
    uint32_t last_tick = SysTick->VAL; //��ȡ��ǰϵͳtick
    
    //SYSTICK ��һ���ݼ��Ķ�ʱ��
    uint32_t tickNum = 0;
    uint32_t reload_tick = SysTick->LOAD;

    uint32_t current_tick = 0;
    
    uint32_t delta_tick = 0;

    uint32_t timeout_tick = 32 * timeout; //����һ����Ҫ�߶���tick

    tickNum = 0;
    

    while (tickNum <= timeout_tick)
    {
        current_tick = SysTick->VAL; //��ȡ��ǰʱ��

        if (current_tick > last_tick) //������
        {
            delta_tick = reload_tick + last_tick - current_tick;
        }
        else //û������
        {
            //��Ϊ����һ���ݼ��ļĴ�����������last_tick��ȥ��ǰ��ʱ����current_tick��ʾ�߹���tick����
            delta_tick = last_tick - current_tick;
        }

        tickNum += delta_tick;

        last_tick = current_tick;
    }
}
