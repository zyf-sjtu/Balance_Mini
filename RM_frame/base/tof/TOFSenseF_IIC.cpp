/**
 ******************************************************************************
 * @file    TOFSenseF_IIC.cpp/h
 * @brief   TOFSenseF_IIC driver. TOF传感器驱动
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */
#include "TOFSenseF_IIC.h"

uint8_t iic_read_buff[50];//读取缓存数组
uint8_t iic_write_buff[50];//写入缓存数组
uint16_t iic_test_count = 0;
uint16_t iic_test_i = 0;
tofsense_f_output_t tofsense_f_output_data;//解码后存放TOF输出数据的结构体

/************************************************
函数名称 ： TOF_IIC_Update_Distance
函数功能 ： 通过IIC读取所有寄存器信息并进行解码，将解码后变量存入结构体成员变量中
参    数 ： *pdata缓存数组的指针  *para数据结构体地址 slave_addr从机地址
返 回 值 ： mark 是否读取和解码成功标志位，0错误，1正确
调用方法：例如：IIC_Unpack_Data(iic_read_buff,&output,ADDR_SLAVE);
*************************************************/
uint8_t TOF_IIC_Update_Distance(tofsense_f_output_t *para, uint8_t *pdata, uint8_t slave_addr) {
  uint8_t mark = 0;//是否读取和解码成功标志位
  if (!pdata)pdata = iic_read_buff;
  mark = TOF_IIC_Read_N_Byte(TOF_ADDR_DIS, TOF_SIZE_DIS+TOF_SIZE_DIS_STATUS, pdata, slave_addr);
  if (mark) {
    para->dis = (float) (((uint32_t) pdata[0]) | ((uint32_t) pdata[1] << 8) |
                         ((uint32_t) pdata[2] << 16) |
                         ((uint32_t) pdata[3] << 24)) / 1000.0f;
    para->dis_status = (uint16_t) (((uint16_t) pdata[4]) |
                                   ((uint16_t) pdata[5] << 8));
  }

  return mark;//返回标志位
}

/************************************************
函数名称 ： TOF_IIC_Update_Measurement
函数功能 ： 通过IIC读取所有寄存器信息并进行解码，将解码后变量存入结构体成员变量中
参    数 ： *pdata缓存数组的指针  *para数据结构体地址 slave_addr从机地址
返 回 值 ： mark 是否读取和解码成功标志位，0错误，1正确
调用方法：例如：IIC_Unpack_Data(iic_read_buff,&output,ADDR_SLAVE);
*************************************************/
uint8_t TOF_IIC_Update_Measurement(tofsense_f_output_t *para, uint8_t *pdata, uint8_t slave_addr) {
  uint8_t mark = 0;//是否读取和解码成功标志位
  if (!pdata)pdata = iic_read_buff;
  mark = TOF_IIC_Read_N_Byte(TOF_ADDR_SYSTEM_TIME, TOF_MEASURE_DATA_SIZE, pdata, slave_addr);
  if (mark) {
    para->system_time = (float) (((uint32_t) pdata[TOF_ADDR_SYSTEM_TIME_RELA]) |
                                 ((uint32_t) pdata[TOF_ADDR_SYSTEM_TIME_RELA + 1] << 8) |
                                 ((uint32_t) pdata[TOF_ADDR_SYSTEM_TIME_RELA + 2] << 16) |
                                 ((uint32_t) pdata[TOF_ADDR_SYSTEM_TIME_RELA + 3] << 24)) / 1000.0f;
    para->dis = (float) (((uint32_t) pdata[TOF_ADDR_DIS_RELA]) | ((uint32_t) pdata[TOF_ADDR_DIS_RELA + 1] << 8) |
                         ((uint32_t) pdata[TOF_ADDR_DIS_RELA + 2] << 16) |
                         ((uint32_t) pdata[TOF_ADDR_DIS_RELA + 3] << 24)) / 1000.0f;
    para->dis_status = (uint16_t) (((uint16_t) pdata[TOF_ADDR_DIS_STATUS_RELA]) |
                                   ((uint16_t) pdata[TOF_ADDR_DIS_STATUS_RELA + 1] << 8));
    para->signal_strength = (uint16_t) (((uint16_t) pdata[TOF_ADDR_SIGNAL_STRENGTH_RELA]) |
                                        ((uint16_t) pdata[TOF_ADDR_SIGNAL_STRENGTH_RELA + 1] << 8));
    para->range_precision = pdata[TOF_ADDR_RANGE_PRECISION_RELA];
  }

  return mark;//返回标志位
}

/************************************************
函数名称 ： TOF_IIC_Update_All_Data
函数功能 ： 通过IIC读取所有寄存器信息并进行解码，将解码后变量存入结构体成员变量中
参    数 ： *pdata缓存数组的指针  *para数据结构体地址  slave_addr从机地址
返 回 值 ： mark 是否读取和解码成功标志位，0错误，1正确
调用方法：例如：IIC_Unpack_Data(iic_read_buff,&output,ADDR_SLAVE);
*************************************************/
uint8_t TOF_IIC_Update_All_Data(tofsense_f_output_t *para, uint8_t *pdata, uint8_t slave_addr) {
  uint8_t mark = 0;//是否读取和解码成功标志位
  if (!pdata)pdata = iic_read_buff;
  mark = TOF_IIC_Get_All_Register_Data(pdata, slave_addr);
  if (mark == 1) {
    para->interface_mode = pdata[TOF_ADDR_MODE] & 0x07;
    para->id = pdata[TOF_ADDR_ID];
    para->uart_baudrate = (uint32_t) (((uint32_t) pdata[TOF_ADDR_UART_BAUDRATE]) |
                                      ((uint32_t) pdata[TOF_ADDR_UART_BAUDRATE + 1] << 8) |
                                      ((uint32_t) pdata[TOF_ADDR_UART_BAUDRATE + 2] << 16) |
                                      ((uint32_t) pdata[TOF_ADDR_UART_BAUDRATE + 3] << 24));
    para->system_time = (float) (((uint32_t) pdata[TOF_ADDR_SYSTEM_TIME]) |
                                 ((uint32_t) pdata[TOF_ADDR_SYSTEM_TIME + 1] << 8) |
                                 ((uint32_t) pdata[TOF_ADDR_SYSTEM_TIME + 2] << 16) |
                                 ((uint32_t) pdata[TOF_ADDR_SYSTEM_TIME + 3] << 24)) / 1000.0f;
    para->dis = (float) (((uint32_t) pdata[TOF_ADDR_DIS]) | ((uint32_t) pdata[TOF_ADDR_DIS + 1] << 8) |
                         ((uint32_t) pdata[TOF_ADDR_DIS + 2] << 16) |
                         ((uint32_t) pdata[TOF_ADDR_DIS + 3] << 24)) / 1000.0f;
    para->dis_status = (uint16_t) (((uint16_t) pdata[TOF_ADDR_DIS_STATUS]) |
                                   ((uint16_t) pdata[TOF_ADDR_DIS_STATUS + 1] << 8));
    para->signal_strength = (uint16_t) (((uint16_t) pdata[TOF_ADDR_SIGNAL_STRENGTH]) |
                                        ((uint16_t) pdata[TOF_ADDR_SIGNAL_STRENGTH + 1] << 8));
    para->range_precision = pdata[TOF_ADDR_RANGE_PRECISION];
  }

  return mark;//返回标志位
}

/************************************************
函数名称 ： TOF_IIC_Change_Mode_To_UART
函数功能 ： 通过IIC将通信模式改为UART模式
参    数 ： slave_addr从机地址
返 回 值 ： mark 操作是否正确标志位，0错误，1正确
*************************************************/
uint8_t TOF_IIC_Change_Mode_To_UART(uint8_t slave_addr) {
  uint8_t mark = 0;//操作是否正确标志位

  mark = TOF_IIC_Write_One_Byte(TOF_ADDR_MODE, IIC_CHANGE_TO_UART_DATA, TOF_ADDR_SLAVE);

  return mark;//返回标志位
}

/************************************************
函数名称 ： TOF_IIC_Get_All_Register_Data
函数功能 ： 通过IIC按顺序读取所有寄存器的数据并存入指定数组
参    数 ： *pdata缓存数组的指针  slave_addr从机地址
返 回 值 ： mark 操作是否正确标志位，0错误，1正确
*************************************************/
uint8_t TOF_IIC_Get_All_Register_Data(uint8_t *pdata, uint8_t slave_addr) {
  uint8_t mark = 0;//操作是否正确标志位

  mark = TOF_IIC_Read_N_Byte(0x00, TOF_REGISTER_TOTAL_SIZE, pdata, slave_addr);

  return mark == HAL_OK;//返回标志位
}

/************************************************
函数名称 ： TOF_IIC_Read_One_Byte
函数功能 ： 通过IIC读取指定地址的单个字节数据
参    数 ： Addr 需要读取的寄存器地址  slave_addr从机地址
返 回 值 ： data 该地址的数据
*************************************************/
uint8_t TOF_IIC_Read_One_Byte(uint8_t Addr, uint8_t slave_addr) {
  uint8_t res;
#if (defined TOF_I2C)
  HAL_I2C_Mem_Read_IT(TOF_I2C, slave_addr, Addr, I2C_MEMADD_SIZE_8BIT,
                   &res, 1);
#endif
  return res == HAL_OK;
}

/************************************************
函数名称 ： TOF_IIC_Read_N_Byte
函数功能 ： 通过IIC读取指定地址的N个字节数据，存入指定的数组中
参    数 ： Addr 需要读取的寄存器起始地址    num读取的字节数    *pdata读取缓存数组的指针  slave_addr从机地址
返 回 值 ： mark 操作是否正确标志位，0错误，1正确
*************************************************/
uint8_t TOF_IIC_Read_N_Byte(uint8_t Addr, uint8_t num, uint8_t *pdata, uint8_t slave_addr) {
  uint8_t res;
#if (defined TOF_I2C)
  res = HAL_I2C_Mem_Read_IT(TOF_I2C, slave_addr, Addr, I2C_MEMADD_SIZE_8BIT,
                         pdata, num);
#endif
  return res == HAL_OK;
}

/************************************************
函数名称 ： TOF_IIC_Write_One_Byte
函数功能 ： 通过IIC写入指定地址单个字节数据
参    数 ： Addr 需要写入的寄存器地址    data 写入该地址的数据  slave_addr从机地址
返 回 值 ：mark 操作是否正确标志位，0错误，1正确
*************************************************/
uint8_t TOF_IIC_Write_One_Byte(uint8_t Addr, uint8_t data, uint8_t slave_addr) {
  uint8_t res;
#if (defined TOF_I2C)
  HAL_I2C_Mem_Write_IT(TOF_I2C, slave_addr, Addr, I2C_MEMADD_SIZE_8BIT,
                    &data, 1);
#endif
  return res == HAL_OK;

}

/************************************************
函数名称 ： TOF_IIC_Write_N_Byte
函数功能 ： 通过IIC写入指定地址多个字节数据
参    数 ： Addr 需要开始写入的寄存器地址    num写入的字节数    *pdata写入缓存数组的指针  slave_addr从机地址
返 回 值 ：mark 操作是否正确标志位，0错误，1正确
*************************************************/
uint8_t TOF_IIC_Write_N_Byte(uint8_t Addr, uint8_t num, uint8_t *pdata, uint8_t slave_addr) {
  uint8_t res;
#if (defined TOF_I2C)
  res = HAL_I2C_Mem_Write_IT(TOF_I2C, slave_addr, Addr, I2C_MEMADD_SIZE_8BIT,
                          pdata, num);
#endif
  return res == HAL_OK;

}