/*
    sht20传感器驱动
        sht20.c
        sht20.h
    
    使用方法:
        (1)单独取值(需自主开关机)
            I2C初始化 hal_i2c_init(i2c_bus_id);
            初始化电源脚 sht20_Init();
            开机   sht20_SetPower(POWER_ON);
            软复位 sht20_SoftReset(i2c_bus_id);
            取温度 sht20_GetTemprature(&temp);
            关机   sht20_SetPower(POWER_OFF);
            
        (2)取5个值去高低取平均值(自动开关机)
            I2C初始化 hal_i2c_init(i2c_bus_id);
            初始化电源脚 sht20_Init();
            取平均 sht20_GetTempAndHumi_Average(i2c_bus_id, &temp，&humi);
*/

#ifndef _SHT20_H_
#define _SHT20_H_

#include <stdint.h>
#include <stdbool.h>
#include "hal_i2c.h"

#define SHT20_ADDR     0x40//sensor I2C address

#define POWER_ON       0
#define POWER_OFF      1

typedef struct{
    bool code;
    float temp;
    float humi;
}sht20_result_t;

typedef enum {
    TRIG_TEMP_MEASUREMENT_HM   = 0xE3, //命令触发。温度测量. hold master
    TRIG_HUMI_MEASUREMENT_HM   = 0xE5, //命令触发。湿度测量. hold master
    TRIG_TEMP_MEASUREMENT_POLL = 0xF3, //命令触发。温度测量. no hold master
    TRIG_HUMI_MEASUREMENT_POLL = 0xF5, //命令触发。湿度测量. no hold master
    USER_REG_W                 = 0xE6, //命令编写用户寄存器
    USER_REG_R                 = 0xE7, //命令读取用户寄存器
    SOFT_RESET                 = 0xFE  //命令软复位
} sht20_command;


/* 初始化 */
extern void sht20_Init(void);
/* 读取温度 */
extern bool sht20_GetTemprature(i2c_bus_id id, float *temp);
/* 读取湿度 */
extern bool sht20_GetHumility(i2c_bus_id id, float *humi);
/* 开关机 */
extern void sht20_SetPower(bool en);
/* 软复位 */
extern void sht20_SoftReset(i2c_bus_id id);
/* 取温湿度值，取5次求平均 */
extern bool sht20_GetTempAndHumi_Average(i2c_bus_id id, float *temp, float *humi);

#endif

