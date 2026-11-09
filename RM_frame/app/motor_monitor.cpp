/**
 ******************************************************************************
 * @file    motor_monitor.cpp/h
 * @brief   Motor parameter, id config and management. 电机参数id配置和统一管理
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/motor_monitor.h"
#include "lib/arm_math/arm_math.h"

// Motor parameter config
// 电机参数配置

// Wheel motor 驱动轮电机
const PID wheel_l_spid(10, 0, 1, 0, 16384);
const PID wheel_r_spid(10, 0, 1, 0, 16384);

float wheelModel(const float& torque, const float& speed = 0) {
  return torque * 3.7e3f;  // 8192*19.2/(0.3*14) ~= 3.7e3
}
const float wheel_kf_K[2] = {0.6, 30};
Motor WML(Motor::M3508, -1.f, Motor::TORQUE, // type, ratio, method
          PID(), wheel_l_spid, // ppid, spid
          true, Motor::KFParam_t(2, 1e4, 1, 0.75, 50),  // kf
          wheelModel);                                  // model
Motor WMR(Motor::M3508, 1.f, Motor::TORQUE, // type, ratio, method
          PID(), wheel_r_spid, // ppid, spid
          true, Motor::KFParam_t(2, 1e4, 1, 0.75, 50),  // kf
          wheelModel);                                  // model
// Leg joint motor 腿部关节电机
// L1: 左前, L2: 左后, R1: 右前, R2: 右后
const PID joint_ppid(0, 0, 0, 0, 0);
const PID joint_spid(0.2, 0, 0.5, 0, 10);
Motor JML1(Motor::MIT, -1, Motor::TORQUE,   // type, ratio, method
           joint_ppid, joint_spid, true);  // ppid, spid, use kf
Motor JML2(Motor::MIT, 1, Motor::TORQUE,  // type, ratio, method
           joint_ppid, joint_spid, true);  // ppid, spid, use kf
Motor JMR1(Motor::MIT, 1, Motor::TORQUE,  // type, ratio, method
           joint_ppid, joint_spid, true);  // ppid, spid, use kf
Motor JMR2(Motor::MIT, -1, Motor::TORQUE,   // type, ratio, method
           joint_ppid, joint_spid, true);  // ppid, spid, use kf

// Gimbal motor 云台电机
Motor GMY(Motor::GM6020, 1, Motor::POSITION_SPEED,   // type, ratio, method
          PID(18, 0.5, 200, 10, 1500, 0.25),                  // ppid
          PID(640, 0, 10, 500, 30000),               // spid
          false);                                    // use kf
Motor GMP(Motor::GM6020, 1, Motor::POSITION_SPEED,  // type, ratio, method
          PID(26, 0.3, 400, 12, 1800, 0.25),                  // ppid
          PID(160, 0, 10, 500, 30000),               // spid
          false);                                    // use kf


// Friction wheel motors
const PID fricl_spid(5, 0, 11, 0, 16384, 1);
const PID fricr_spid(5, 0, 10, 0, 16384, 1);
Motor FRICL(Motor::M3508, 1, Motor::SPEED,  // type, ratio, method
            PID(), PID(fricl_spid));         // ppid, spid
Motor FRICR(Motor::M3508, 1, Motor::SPEED,  // type, ratio, method
            PID(), PID(fricr_spid));         // ppid, spid

// Stir motor
Motor STIR(Motor::M2006, -36, Motor::POSITION_SPEED,  // type, ratio, method
           PID(20, 0.1, 10, 10, 2500),                // ppid
           PID(60, 0.1, 200, 1000, 10000));           // spid

// Other motors
Motor m1(Motor::M3508, 1, Motor::POSITION_SPEED,  // type, ratio, method
         PID(50, 0.02, 50, 200, 36000),           // ppid
         PID(20, 0.02, 50, 200, 16384));          // spid

// DJI Motor id config, M3508/M2006: 1~8, GM6020: 5~11
// DJI电机ID配置，M3508，M2006可配置范围为1~8，GM6020可配置范围为5~11
Motor* can1_dji_motor[11] = {
    &WML,  // id:1
    &WMR,  // id:2
    nullptr,  // id:3
    nullptr,  // id:4
    nullptr,    // id:5
    nullptr,     // id:6
    nullptr,   // id:7
    nullptr,   // id:8
    nullptr,  // id:9
    nullptr,  // id:10
    nullptr   // id:11
};
Motor* can2_dji_motor[11] = {
    nullptr,  // id:1
    nullptr,  // id:2
    nullptr,  // id:3
    nullptr,  // id:4
    nullptr,     // id:5
    nullptr,  // id:6
    nullptr,     // id:7
    nullptr,     // id:8
    nullptr,  // id:9
    nullptr,  // id:10
    nullptr   // id:11
};

DJIMotorDriver dji_motor_driver(can1_dji_motor, can2_dji_motor);

// DM8009电机参数
namespace dm4310 {
  // position limit(rad)
  const float p_min = -12.566f;
  const float p_max = 12.566f;
  // velocity limit(rad/s)
  const float v_min = -30.f;
  const float v_max = 30.f;
  // PD param limit
  const float kp_min = 0;
  const float kp_max = 500;
  const float kv_min = 0;
  const float kv_max = 5;
  // feedforward torque limit(N·m)
  const float t_ff_min = -10.f;
  const float t_ff_max = 10.f;
  // feedback torque limit(N·m)
  const float t_min = -10.f;
  const float t_max = 10.f;
}  // namespace dm4310

// MIT协议电机
MITMotorDriver mit_motor_driver[] = {
  MITMotorDriver(&JML1, &hcan2, 0x11, 0x01, dm4310::p_min, dm4310::p_max,
                 dm4310::v_min, dm4310::v_max, dm4310::kp_min, dm4310::kp_max,
                 dm4310::kv_min, dm4310::kv_max, dm4310::t_ff_min,
                 dm4310::t_ff_max, dm4310::t_min, dm4310::t_max),
  MITMotorDriver(&JML2, &hcan2, 0x12, 0x02, dm4310::p_min, dm4310::p_max,
                 dm4310::v_min, dm4310::v_max, dm4310::kp_min, dm4310::kp_max,
                 dm4310::kv_min, dm4310::kv_max, dm4310::t_ff_min,
                 dm4310::t_ff_max, dm4310::t_min, dm4310::t_max),
  MITMotorDriver(&JMR1, &hcan2, 0x13, 0x03, dm4310::p_min, dm4310::p_max,
                 dm4310::v_min, dm4310::v_max, dm4310::kp_min, dm4310::kp_max,
                 dm4310::kv_min, dm4310::kv_max, dm4310::t_ff_min,
                 dm4310::t_ff_max, dm4310::t_min, dm4310::t_max),
  MITMotorDriver(&JMR2, &hcan2, 0x15, 0x05, dm4310::p_min, dm4310::p_max,
                 dm4310::v_min, dm4310::v_max, dm4310::kp_min, dm4310::kp_max,
                 dm4310::kv_min, dm4310::kv_max, dm4310::t_ff_min,
                 dm4310::t_ff_max, dm4310::t_min, dm4310::t_max),
};

// 电机停转速度上限(dps)
const float motor_stop_rotate_speed_thres = 1200;

// Stop and shut off all motors
// 所有电机先停转再断电
void allMotorsStopShutoff(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      if (fabs(can1_dji_motor[i]->motor_data_.rotate_speed) >
          motor_stop_rotate_speed_thres) {
        can1_dji_motor[i]->mode_ = Motor::STOP;
      } else {
        can1_dji_motor[i]->mode_ = Motor::POWEROFF;
      }
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      if (fabs(can2_dji_motor[i]->motor_data_.rotate_speed) >
          motor_stop_rotate_speed_thres) {
        can2_dji_motor[i]->mode_ = Motor::STOP;
      } else {
        can2_dji_motor[i]->mode_ = Motor::POWEROFF;
      }
    }
  }
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->mode_ = Motor::POWEROFF;
  }
}

// Shut off all motors
// 所有电机断电
void allMotorsShutOff(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::POWEROFF;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::POWEROFF;
    }
  }
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->mode_ = Motor::POWEROFF;
  }
}

// Stop all motors
// 所有电机停转
void allMotorsStop(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::STOP;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::STOP;
    }
  }
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->mode_ = Motor::STOP;
  }
}

// Startup all motors
// 所有电机启动
void allMotorsOn(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::WORKING;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::WORKING;
    }
  }
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->mode_ = Motor::WORKING;
  }
}

// Reset all motors.
// 电机统一初始化
void allMotorsInit(void) {
  dji_motor_driver.idConfig();
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr) {
      can1_dji_motor[i]->reset();
    }
    if (can2_dji_motor[i] != nullptr) {
      can2_dji_motor[i]->reset();
    }
  }
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->reset();
    mit_motor_driver[i].setCmd(mitmotor::MOTOR_MODE);
  }
}

// Handle all motors. Called in motorTask
// 电机统一处理，在motorTask中调用
void allMotorsHandle(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr) {
      can1_dji_motor[i]->handle();
    }
    if (can2_dji_motor[i] != nullptr) {
      can2_dji_motor[i]->handle();
    }
  }
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->handle();
    if (!mit_motor_driver[i].motor_->connect_.check()) {
      mit_motor_driver[i].setCmd(mitmotor::MOTOR_MODE);
    }
  }
}

// Check CAN channel and id of received CAN message
// 校验接收信息的CAN通道和ID，调用对应回调函数
void motorsCanRxMsgHandle(CAN_HandleTypeDef* hcan,
                          CAN_RxHeaderTypeDef rx_header, uint8_t* rx_data) {
  if (dji_motor_driver.canRxMsgCheck(hcan, rx_header)) {
    dji_motor_driver.canRxMsgCallback(hcan, rx_header, rx_data);
  }
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    if (mit_motor_driver[i].canRxMsgCheck(hcan, rx_header)) {
      mit_motor_driver[i].canRxMsgCallback(hcan, rx_header, rx_data);
    }
  }
}
