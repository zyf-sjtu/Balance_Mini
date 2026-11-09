/**
 ******************************************************************************
 * @file    battery_monitor.cpp/h
 * @brief   Battery voltage monitor. 电池电压监测
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/battery_monitor.h"

namespace {
// 硬件参数
constexpr float kDividerRatio = 10.0f;    // 分压比
constexpr float kAdcRefVoltage = 3.3f;    // ADC 参考电压(V)
constexpr float kAdcFullScale = 4095.0f;  // 12bit

// 低压保护阈值与恢复阈值(V)，二者之差构成迟滞
constexpr float kLowThreshold = 22.0f;
constexpr float kRecoverThreshold = 22.5f;
}  // namespace

void BatteryMonitor::init() {
  HAL_ADC_Start(hadc_);
}

void BatteryMonitor::handle() {
  HAL_ADC_Start(hadc_);
  if (HAL_ADC_PollForConversion(hadc_, 10) != HAL_OK) {
    HAL_ADC_Stop(hadc_);
    return;
  }
  const float adc_volt =
      HAL_ADC_GetValue(hadc_) * kAdcRefVoltage / kAdcFullScale;
  HAL_ADC_Stop(hadc_);

  // 滑动平均，抑制加速/刹车时的电压跌落毛刺
  samples_[index_] = adc_volt * kDividerRatio;
  index_ = (index_ + 1) % kFilterSize;
  if (count_ < kFilterSize) {
    ++count_;
  }

  float sum = 0.0f;
  for (uint8_t i = 0; i < count_; ++i) {
    sum += samples_[i];
  }
  voltage_ = sum / count_;

  // 迟滞判断：低于下限置位，高于恢复阈值才清除
  if (voltage_ < kLowThreshold) {
    low_ = true;
  } else if (voltage_ > kRecoverThreshold) {
    low_ = false;
  }
}
