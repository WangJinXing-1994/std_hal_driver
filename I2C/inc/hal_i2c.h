/*
 * @Author chenjk
 * @Date 2020-05-21 16:05:22
 * @LastEditTime 2020-06-18 15:39:10
 * @FilePath \rtt-hc32-sdk\hal\inc\hal_i2c.h
 * @Description 
 *      i2c总线的操作，MCU作为I2C总线的主机
 */ 

#ifndef HAL_I2C_H
#define HAL_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup HAL_I2C
 * @brief HAL I2C API
 * @{
 */


#include <stdint.h>

/**
 * @brief I2C总线定义
 */
typedef enum{
    I2C_BUS_0,
    I2C_BUS_1,
    I2C_BUS_2,
    I2C_BUS_3,
}i2c_bus_id;

#define I2C_WR  0   /*!< I2C写 */
#define I2C_RD  1   /*!< I2C读 */

/**
 * @brief I2C消息结构
 */
typedef struct 
{
    uint16_t salve_addr;    /*!< 从机地址 */
    uint8_t wr_flag;        /*!< 读写标志位 */
    uint8_t* buf;           /*!< 读/写的数据的地址 */
    uint16_t len;           /*!< 读/写的数据的长度 */
}i2c_msg_t;

/**
 * @brief I2C总线初始化
 * @param i2c_bus I2C总线的ID
 */
void hal_i2c_init(i2c_bus_id i2c_bus);

/**
 * @brief I2C deinit
 * @param i2c_bus I2C总线的ID
 */
void hal_i2c_deinit(i2c_bus_id i2c_bus);

/**
 * @brief I2C 总线数据传输，读/写都可以调用这个函数
 * @param i2c_bus I2C总线的ID
 * @param i2c_msg i2c消息
 * @param num i2c_msg[]的元素个数
 * @details 此函数会先发送i2c_start,再进行发送i2c数据，当消息都发送完之后，最后发送i2c_stop
 * @retval 0:操作成功
 * @retval -1:操作失败
 */
int hal_i2c_transfer(i2c_bus_id i2c_bus,i2c_msg_t i2c_msg[],uint32_t num);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
