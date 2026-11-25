/**
 ******************************************************************************
 * @file    control.cpp/h
 * @brief   Robot control design. 机器人控制设计（模式/键位）
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/control.h"

#include "battery_monitor.h"
#include "iwdg.h"

#include "app/chassis.h"
#include "app/imu_monitor.h"
#include "app/motor_monitor.h"
#include "base/bsp/bsp_buzzer.h"
#include "base/bsp/bsp_led.h"
#include "base/common/math.h"
#include "base/remote/remote.h"

void iwdgHandler(bool iwdg_refresh_flag);
void robotPowerStateFSM(bool stop_flag);
void robotReset(void);
bool robotStartup(void);
void robotControl(void);
void boardLedHandle(void);

extern RC rc;

WBRChassis chassis(&WML, &WMR, &JML1, &JML2, &JMR1, &JMR2, &board_imu);
BoardLed led;
BoardBuzzer buzzer;

// 上电状态
enum RobotPowerState_e {
  STOP = 0,
  STARTUP = 1,
  WORKING = 2,
} robot_state;

// 初始化标志
bool robot_init_flag = false;
uint32_t boot_tick = 0;

// 遥控器状态记录
RC::RCSwitch last_rc_switch;
RC::RCChannel last_rc_channel;
RCKey last_key;

// 遥控器控制参数
namespace rcctrl {
const float chassis_speed_rate = 4.8e-3f;
const float chassis_rotate_rate = 8e-6f;
const float leg_length_rate = 5e-7f;
const float chassis_gyro_speed = -3.0f * PI;
const float gimbal_rate = 6e-4f;
const float chassis_yaw_speed_rate = 2e-6f;
} // namespace rcctrl

// 控制初始化
void controlInit(void) {
  led.init();
  chassis.init();
  buzzer.init();
  robot_state = STOP;
  boot_tick = HAL_GetTick();
}

void boardBuzzerHandle();
// 控制主循环
void controlLoop(void) {
  //遥控器断联触发看门狗
  iwdgHandler(rc.connect_.check());
  robotPowerStateFSM(!rc.connect_.check() || rc.switch_.r == RC::DOWN ||
                     battery.isLow());
  if (robot_state == STOP) {
    allMotorsStopShutoff();
    robotReset();
  } else if (robot_state == STARTUP) {
    allMotorsOn(); // 电机上电
    robot_init_flag = robotStartup(); // 开机状态判断
  } else if (robot_state == WORKING) {
    allMotorsOn(); // 电机上电
    robotControl(); // 机器人控制
  }
  //设定目标值
  chassis.handle();
  boardLedHandle();
  boardBuzzerHandle();

  // 记录遥控器状态
  last_rc_switch = rc.switch_;
  last_rc_channel = rc.channel_;
  last_key = rc.key_;
}

// IWDG处理，true持续刷新，false进入STOP状态并停止刷新
void iwdgHandler(bool iwdg_refresh_flag) {
  if (!iwdg_refresh_flag) {
    robot_state = STOP;
  } else {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

// STOP(断电，安全模式)/开机/正常运行 状态机
void robotPowerStateFSM(bool stop_flag) {
  if (robot_state == STOP) {
    if (!stop_flag) {
      robot_state = STARTUP;
    }
  } else if (robot_state == STARTUP) {
    if (stop_flag) {
      robot_state = STOP;
    } else if (robot_init_flag) {
      // 初始化/复位完成
      robot_state = WORKING;
    }
  } else if (robot_state == WORKING) {
    if (stop_flag) {
      robot_state = STOP;
    }
  }
}

// 重置各功能状态
void robotReset(void) {
  robot_init_flag = false;
  chassis.mode_ = WBRChassis::STOP;
}

// 开机上电启动处理
bool robotStartup(void) {
  // 初始化底盘
  chassis.mode_ = WBRChassis::DIRECT;
  return true;
}

// 机器人控制
void robotControl(void) {
  // TODO 档位控制
  // 遥控器挡位左上右上
  if (rc.switch_.l == RC::UP && rc.switch_.r == RC::UP) {
    chassis.mode_ = WBRChassis::DIRECT;
    chassis.targetS_D1() = rc.channel_.r_col * rcctrl::chassis_speed_rate;
    chassis.targetPhi() -= rc.channel_.l_row * rcctrl::chassis_yaw_speed_rate;
    chassis.targetL() -= rc.channel_.dial_wheel * rcctrl::leg_length_rate;
  }
  // --你可以在这个地方利用不同的遥控器挡位去开发不同的功能--
  // --例如跳跃，陀螺，爬行等等--
  // 遥控器挡位左中右上
  else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::UP) {
    chassis.mode_ = WBRChassis::STOP;
  }
  // 遥控器挡位左下右上
  else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::UP) {
    chassis.mode_ = WBRChassis::STOP;
  }
  // 遥控器挡位左上右中
  else if (rc.switch_.l == RC::UP && rc.switch_.r == RC::MID) {
    chassis.mode_ = WBRChassis::STOP;
  }
  // 遥控器挡位左中右中
  else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::MID) {
    chassis.mode_ = WBRChassis::STOP;
  }
  // 遥控器挡位左下右中
  else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::MID) {
    chassis.mode_ = WBRChassis::STOP;
  }
  else {
    chassis.mode_ = WBRChassis::STOP;
  }
}

// 板载LED指示灯效果
void boardLedHandle(void) {
#ifdef DBC
  if (robot_state == STOP) {
    led.setColor(255, 0, 0); // red
    led.setModeOn();
  } else if (robot_state == STARTUP) {
    led.setColor(150, 150, 0); // yellow
    led.setModeBreath();
  } else if (robot_state == WORKING) {
    led.setColor(0, 0, 255); // blue
    if (rc.switch_.l == RC::UP && rc.switch_.r == RC::UP) {
      // 遥控器挡位左上右上
      led.setModeBlink(1);
    } else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::UP) {
      // 遥控器挡位左中右上
      led.setModeBlink(2);
    } else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::UP) {
      // 遥控器挡位左下右上
      led.setModeBlink(3);
    } else if (rc.switch_.l == RC::UP && rc.switch_.r == RC::MID) {
      // 遥控器挡位左上右中
      led.setModeBlink(4);
    } else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::MID) {
      // 遥控器挡位左中右中
      led.setModeBlink(5);
    } else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::MID) {
      // 遥控器挡位左下右中
      led.setModeBlink(6);
    }
  }
  led.handle();
#elif defined DBA
  if (robot_state == STOP) {
    led.setLED(true, false, 0); // red
  } else if (robot_state == STARTUP) {
    led.setLED(true, true, 0); // red+green
  } else if (robot_state == WORKING) {
    if (rc.switch_.l == RC::UP && rc.switch_.r == RC::UP) {
      // 遥控器挡位左上右上
      led.setLED(false, true, 1); // green
    } else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::UP) {
      // 遥控器挡位左中右上
      led.setLED(false, true, 2); // green
    } else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::UP) {
      // 遥控器挡位左下右上
      led.setLED(false, true, 3); // green
    } else if (rc.switch_.l == RC::UP && rc.switch_.r == RC::MID) {
      // 遥控器挡位左上右中
      led.setLED(false, true, 4); // green
    } else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::MID) {
      // 遥控器挡位左中右中
      led.setLED(false, true, 5); // green
    } else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::MID) {
      // 遥控器挡位左下右中
      led.setLED(false, true, 6); // green
    }
  }
#endif
}

void playStartUpMusic() {
  uint16_t beat_time = 500;
  uint32_t t = HAL_GetTick() - boot_tick;
  if (t < 1 * beat_time) {
    buzzer.SetMusicNote(M1);
  } else if (t < 2 * beat_time) {
    buzzer.SetMusicNote(M2);
  } else if (t < 3 * beat_time) {
    buzzer.SetMusicNote(M3);
  } else if (t < 4 * beat_time) {
    buzzer.SetMusicNote(M1);
  } else if (t < 5 * beat_time) {
    buzzer.SetMusicNote(M2);
  } else if (t < 5.8 * beat_time) {
    buzzer.SetMusicNote(M5);
  } else if (t < 6.0 * beat_time) {
    buzzer.Mute();
  } else if (t < 7 * beat_time) {
    buzzer.SetMusicNote(M5);
  } else if (t < 8 * beat_time) {
    buzzer.Mute();
  } else if (t < 9 * beat_time) {
    buzzer.SetMusicNote(M3);
  } else if (t < 9.5 * beat_time) {
    buzzer.SetMusicNote(M6);
  } else if (t < 9.75 * beat_time) {
    buzzer.Mute();
  } else if (t < 9.9 * beat_time) {
    buzzer.SetMusicNote(M6);
  } else if (t < 10 * beat_time) {
    buzzer.Mute();
  } else if (t < 10.9 * beat_time) {
    buzzer.SetMusicNote(M6);
  } else if (t < 11 * beat_time) {
    buzzer.Mute();
  } else if (t < 12 * beat_time) {
    buzzer.SetMusicNote(M6);
  } else if (t < 12.9 * beat_time) {
    buzzer.SetMusicNote(M5);
  } else if (t < 13 * beat_time) {
    buzzer.Mute();
  } else if (t < 14 * beat_time) {
    buzzer.SetMusicNote(M5);
  } else if (t < 15 * beat_time) {
    buzzer.SetMusicNote(M3);
  } else {
    buzzer.Mute();
  }
}

void boardBuzzerHandle() {
  if (HAL_GetTick() - boot_tick < 9000) {
    playStartUpMusic();
    buzzer.Handle();
    return;
  }
  if (battery.isLow()) {
    buzzer.SetBuzzer(8, H1);
  } else {
    buzzer.SetBuzzer(0, 0);
  }
  buzzer.Handle();
}
