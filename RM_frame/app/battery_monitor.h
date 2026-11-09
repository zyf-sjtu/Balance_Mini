/**
 ******************************************************************************
 * @file    battery_monitor.cpp/h
 * @brief   Battery voltage monitor. 电池电压监测
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include "adc.h"

// 电池电压监测：ADC 采样 -> 滑动平均 -> 低压迟滞判断
class BatteryMonitor {
 public:
  // 滑动平均点数
  static constexpr uint8_t kFilterSize = 8;

  explicit BatteryMonitor(ADC_HandleTypeDef* hadc) : hadc_(hadc) {}

  // 启动 ADC，开机时调用一次
  void init();
  // 采样一次并更新滤波值，建议 100ms 调用一次
  void handle();

  // 滤波后的电池电压(V)
  float voltage() const { return voltage_; }
  // 是否处于低压保护状态(带迟滞)
  bool isLow() const { return low_; }

 private:
  ADC_HandleTypeDef* hadc_;

  float samples_[kFilterSize] = {};
  uint8_t index_ = 0;
  uint8_t count_ = 0;
  // 首次采样前按低压处理，避免电压未确认就上力
  bool low_ = true;
  float voltage_ = 0.0f;
};

// 电池监测实例(定义于 interface/robot.cpp)
extern BatteryMonitor battery;

#endif  // BATTERY_MONITOR_H
