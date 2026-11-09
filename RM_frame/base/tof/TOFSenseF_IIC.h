/**
 ******************************************************************************
 * @file    TOFSenseF_IIC.cpp/h
 * @brief   TOFSenseF_IIC driver. TOF传感器驱动
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef RM_FRAME_TOFSENSEF_IIC_H
#define RM_FRAME_TOFSENSEF_IIC_H

#include <cstdint>
#include "hardware_config.h"
#include "i2c.h"
#include "TOFSenseF_IIC_reg.h"

typedef struct {
    uint8_t id;//ID
    uint8_t interface_mode;//通信接口模式，0-UART，1-CAN，2-I/O，3-IIC
    uint32_t uart_baudrate;//UART波特率
    float system_time;//系统时间, s
    float dis;//距离, m
    uint16_t dis_status;//距离状态指示，0-invalid，1-valid
    uint16_t signal_strength;//信号强度, the larger, the better
    uint8_t range_precision;//测距精度, cm, the smaller, the better
} tofsense_f_output_t;//TOFSense-F输出的参数结构体

extern uint8_t iic_read_buff[50];//读取缓存数组
extern uint8_t iic_write_buff[50];//写入缓存数组
extern uint16_t iic_test_count;
extern uint16_t iic_test_i;
extern tofsense_f_output_t tofsense_f_output_data;//解码后存放TOF输出数据的结构体

uint8_t TOF_IIC_Update_Distance(tofsense_f_output_t *para, uint8_t *pdata = nullptr,
                                   uint8_t slave_addr = TOF_ADDR_SLAVE);//通过IIC读取仅距离数据并进行解码，将解码后变量存入结构体成员变量中
uint8_t TOF_IIC_Update_Measurement(tofsense_f_output_t *para, uint8_t *pdata = nullptr,
                                   uint8_t slave_addr = TOF_ADDR_SLAVE);//通过IIC读取测量相关数据并进行解码，将解码后变量存入结构体成员变量中
uint8_t TOF_IIC_Update_All_Data(tofsense_f_output_t *para, uint8_t *pdata = nullptr,
                            uint8_t slave_addr = TOF_ADDR_SLAVE);//通过IIC读取所有寄存器信息并进行解码，将解码后变量存入结构体成员变量中
uint8_t TOF_IIC_Change_Mode_To_UART(uint8_t slave_addr = TOF_ADDR_SLAVE);//通过IIC将通信模式改为UART模式
uint8_t TOF_IIC_Get_All_Register_Data(uint8_t *pdata, uint8_t slave_addr = TOF_ADDR_SLAVE);//通过IIC按顺序读取所有寄存器的数据并存入指定数组
uint8_t TOF_IIC_Read_One_Byte(uint8_t Addr, uint8_t slave_addr = TOF_ADDR_SLAVE);//通过IIC读取指定地址的单个字节数据
uint8_t TOF_IIC_Read_N_Byte(uint8_t Addr, uint8_t num, uint8_t *pdata,
                            uint8_t slave_addr = TOF_ADDR_SLAVE);//通过IIC读取指定地址的N个字节数据，存入指定的数组中
uint8_t TOF_IIC_Write_One_Byte(uint8_t Addr, uint8_t data, uint8_t slave_addr = TOF_ADDR_SLAVE);//通过IIC写入指定地址单个字节数据
uint8_t
TOF_IIC_Write_N_Byte(uint8_t Addr, uint8_t num, uint8_t *pdata, uint8_t slave_addr = TOF_ADDR_SLAVE);//通过IIC写入指定地址多个字节数据


#endif //RM_FRAME_TOFSENSEF_IIC_H
