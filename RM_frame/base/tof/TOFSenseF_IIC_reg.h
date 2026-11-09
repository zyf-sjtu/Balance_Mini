/**
 ******************************************************************************
 * @file    TOFSenseF_IIC.cpp/h
 * @brief   TOFSenseF_IIC driver. TOF传感器驱动
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef RM_FRAME_TOFSENSEF_IIC_REG_H
#define RM_FRAME_TOFSENSEF_IIC_REG_H

#define TOF_ADDR_SLAVE (0x08<<1)//7位从机地址=ID+0x08
//#define TOF_ADDR_WRITE_SLAVE (uint8_t)((ADDR_SLAVE<<1)|0x00)//8位从机写地址
//#define TOFSENSE_F_ADDR_READ_SLAVE (uint8_t)((ADDR_SLAVE<<1)|0x01)//8位从机读地址

//寄存器变量大小地址列表（常用）
//#define TOF_REGISTER_SIZE 4//单个寄存器长度
#define TOF_REGISTER_TOTAL_SIZE 48//所有寄存器总共长度
#define TOF_MEASURE_DATA_SIZE 16//测量相关的变量所在寄存器长度

#define TOF_ADDR_MODE 0x0c//模式变量地址
#define TOF_SIZE_MODE 1//模式变量所占的字节数

#define TOF_ADDR_ID 0x0d//ID变量地址
#define TOF_SIZE_ID 1//ID变量所占的字节数

#define TOF_ADDR_UART_BAUDRATE 0x10//UART波特率变量地址
#define TOF_SIZE_UART_BAUDRATE 4//UART波特率变量所占的字节数

#define TOF_ADDR_MEASURE_DATA 0X20//测量相关的变量起始寄存器地址
#define TOF_ADDR_SYSTEM_TIME 0x20//系统时间变量地址
#define TOF_ADDR_SYSTEM_TIME_RELA 0x0//系统时间变量地址，相对0x20偏移值
#define TOF_SIZE_SYSTEM_TIME 4//系统时间变量所占的字节数

#define TOF_ADDR_DIS 0x24//距离变量地址
#define TOF_ADDR_DIS_RELA 0x4//距离变量地址，相对0x20偏移值
#define TOF_SIZE_DIS 4//距离变量所占的字节数

#define TOF_ADDR_DIS_STATUS 0x28//距离状态指示变量地址
#define TOF_ADDR_DIS_STATUS_RELA 0x8//距离状态指示变量地址，相对0x20偏移值
#define TOF_SIZE_DIS_STATUS 2//距离状态指示变量所占的字节数

#define TOF_ADDR_SIGNAL_STRENGTH 0x2a//信号强度变量地址
#define TOF_ADDR_SIGNAL_STRENGTH_RELA 0xa//信号强度变量地址，相对0x20偏移值
#define TOF_SIZE_SIGNAL_STRENGTH 2//信号强度变量所占的字节数

#define TOF_ADDR_RANGE_PRECISION 0x2c//测距精度变量地址
#define TOF_ADDR_RANGE_PRECISION_RELA 0xc//测距精度变量地址，相对0x20偏移值
#define TOF_SIZE_RANGE_PRECISION 1//测距精度变量所占的字节数

#define IIC_CHANGE_TO_UART_DATA 0x40//将通信模式改为UART需要发送的字节数据

#endif //RM_FRAME_TOFSENSEF_IIC_REG_H
