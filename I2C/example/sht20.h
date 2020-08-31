/*
    sht20����������
        sht20.c
        sht20.h
    
    ʹ�÷���:
        (1)����ȡֵ(���������ػ�)
            I2C��ʼ�� hal_i2c_init(i2c_bus_id);
            ��ʼ����Դ�� sht20_Init();
            ����   sht20_SetPower(POWER_ON);
            ��λ sht20_SoftReset(i2c_bus_id);
            ȡ�¶� sht20_GetTemprature(&temp);
            �ػ�   sht20_SetPower(POWER_OFF);
            
        (2)ȡ5��ֵȥ�ߵ�ȡƽ��ֵ(�Զ����ػ�)
            I2C��ʼ�� hal_i2c_init(i2c_bus_id);
            ��ʼ����Դ�� sht20_Init();
            ȡƽ�� sht20_GetTempAndHumi_Average(i2c_bus_id, &temp��&humi);
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
    TRIG_TEMP_MEASUREMENT_HM   = 0xE3, //��������¶Ȳ���. hold master
    TRIG_HUMI_MEASUREMENT_HM   = 0xE5, //�������ʪ�Ȳ���. hold master
    TRIG_TEMP_MEASUREMENT_POLL = 0xF3, //��������¶Ȳ���. no hold master
    TRIG_HUMI_MEASUREMENT_POLL = 0xF5, //�������ʪ�Ȳ���. no hold master
    USER_REG_W                 = 0xE6, //�����д�û��Ĵ���
    USER_REG_R                 = 0xE7, //�����ȡ�û��Ĵ���
    SOFT_RESET                 = 0xFE  //������λ
} sht20_command;


/* ��ʼ�� */
extern void sht20_Init(void);
/* ��ȡ�¶� */
extern bool sht20_GetTemprature(i2c_bus_id id, float *temp);
/* ��ȡʪ�� */
extern bool sht20_GetHumility(i2c_bus_id id, float *humi);
/* ���ػ� */
extern void sht20_SetPower(bool en);
/* ��λ */
extern void sht20_SoftReset(i2c_bus_id id);
/* ȡ��ʪ��ֵ��ȡ5����ƽ�� */
extern bool sht20_GetTempAndHumi_Average(i2c_bus_id id, float *temp, float *humi);

#endif

