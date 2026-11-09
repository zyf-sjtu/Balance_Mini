/**
 ******************************************************************************
 * @file    bsp_buzzer.cpp/h
 * @brief   Buzzer 板载蜂鸣器
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "base/bsp/bsp_buzzer.h"
#include <string.h>
#include "cmsis_os.h"

#ifdef DBC
TIM_HandleTypeDef* board_buzzer_tim = &htim4;
const uint32_t board_buzzer_channel = TIM_CHANNEL_3;
#elif defined DBA
TIM_HandleTypeDef* board_buzzer_tim = &htim12;
const uint32_t board_buzzer_channel = TIM_CHANNEL_1;
#endif

#if (defined DBC) or (defined DBA)

BoardBuzzer::BoardBuzzer(void)
    : htim_(board_buzzer_tim), ch_(board_buzzer_channel) {}

void BoardBuzzer::init(void) {
  HAL_TIM_PWM_Start(htim_, ch_);
}

void BoardBuzzer::SetBuzzer(uint8_t times, uint16_t note) {
  if(times != 0 || HAL_GetTick() - start_tick > 3000) {
    start_tick = HAL_GetTick();
    note_ = note;
    times_ = times > 8 ? 8 : times;
  }
}

void BoardBuzzer::SetMusicNote(uint16_t note) {
  times_ = 9;
  note_ = note;
}

void BoardBuzzer::Mute() {
  times_ = 10;
}


void BoardBuzzer::Handle() {
  uint32_t current_tick = HAL_GetTick();
  if (times_ == 9) {
    __HAL_TIM_SET_AUTORELOAD(htim_, 1e6 / note_);
    __HAL_TIM_SET_COMPARE(htim_, ch_, 1e5 / note_);
    return;
  }
  if (times_ == 10) {
    __HAL_TIM_SET_AUTORELOAD(htim_, 0);
    return;
  }
  if (current_tick > period_start_tick + 1000) {
    period_start_tick = current_tick;
  } else if (current_tick > period_start_tick + times_ * 125) {
    __HAL_TIM_SET_AUTORELOAD(htim_, 0);
  } else if ((current_tick - period_start_tick) % 125 < 62) {
    __HAL_TIM_SET_AUTORELOAD(htim_, 1e6 / note_);
    __HAL_TIM_SET_COMPARE(htim_, ch_, 1e5 / note_);
  } else {
    __HAL_TIM_SET_AUTORELOAD(htim_, 0);
  }
}

#endif
