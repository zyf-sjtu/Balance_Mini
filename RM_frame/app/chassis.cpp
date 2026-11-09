/**
 ******************************************************************************
 * @file    chassis.cpp/h
 * @brief   WBR chassis control. 双足轮腿底盘控制
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/chassis.h"
#include "base/common/math.h"

#include <cmath>

WBRChassis::WBRChassis(Motor* wml, Motor* wmr, Motor* jml1, Motor* jml2,
                       Motor* jmr1, Motor* jmr2, IMU* chassis_imu)
  : wml_(wml),
    wmr_(wmr),
    jml1_(jml1),
    jml2_(jml2),
    jmr1_(jmr1),
    jmr2_(jmr2),
    imu_(chassis_imu) {
  jump_controller_.stage = JumpController_t::JumpStage_e::FINISH;
}

// 底盘初始化
void WBRChassis::init(void) {
  mode_ = WBRChassis::STOP;
  wml_->setFdbSrc(&wml_->kf_data_.x[0], &wml_->kf_data_.x[1]);
  wmr_->setFdbSrc(&wmr_->kf_data_.x[0], &wmr_->kf_data_.x[1]);
  jml1_->setFdbSrc(&jml1_->kf_data_.x[0], &jml1_->kf_data_.x[1]);
  jml2_->setFdbSrc(&jml2_->kf_data_.x[0], &jml2_->kf_data_.x[1]);
  jmr1_->setFdbSrc(&jmr1_->kf_data_.x[0], &jmr1_->kf_data_.x[1]);
  jmr2_->setFdbSrc(&jmr2_->kf_data_.x[0], &jmr2_->kf_data_.x[1]);
  TOF_IIC_Update_All_Data(&tof_.tof_data);
}

// 底盘控制总函数
void WBRChassis::handle(void) {
  // 读取传感器数据(编码器，imu)
  sensor();
  // 观测器(传感器数据->状态)(角度，jacobi矩阵解算)
  observer();
  // 处理目标状态(设定目标X_d)
  handleRef();
  // 控制器
  // 停止状态，条件：STOP模式/开机等待IMU校准/等待轮电机反馈
  if (mode_ == WBRChassis::STOP || HAL_GetTick() < 4e3 ||
      !wml_->connect_.check() || !wmr_->connect_.check() ||
      !jml1_->connect_.check() || !jml2_->connect_.check() ||
      !jmr1_->connect_.check() || !jmr2_->connect_.check()) {
    // 轮电机降速后断电，关节电机断电
    stopController();
  }
  // 爬行模式(仅控制轮电机，不保持平衡)
  else if (mode_ == WBRChassis::CRAWL) {
    crawlController();
  }
  // WBR控制(DIRECT/FOLLOW/GYRO/JUMP)
  else {
    WBRController(); // 控制器
    predictor(); // 状态预测器
  }
}

// 跳跃指令
void WBRChassis::setJump(WBRChassis::JumpCmd_e cmd) {
  // if (cmd == WBRChassis::JumpCmd_e::START) {
  //   // 前一次跳跃完成后可执行下一次跳跃指令
  //   if (jump_controller_.stage == JumpController_t::JumpStage_e::FINISH) {
  //     jump_controller_.stage = JumpController_t::JumpStage_e::START;
  //     jump_controller_.tick = HAL_GetTick();
  //   }
  // } else if (cmd == WBRChassis::JumpCmd::ABORT) {
  //   // 中止跳跃动作，进入常规控制状态
  //   jump_controller_.stage = JumpController_t::JumpStage_e::FINISH;
  // }
}

// WBR控制器
void WBRChassis::WBRController(void) {
  // 运动控制器
  locomotionController();
  // 腿部控制器
  legController();
  // 跳跃控制器
  // jumpController();
  // 腿部关节力矩控制(映射+关节限位)
  jointController();

  // 设置执行电机力矩
  wml_->method_ = Motor::TORQUE;
  wmr_->method_ = Motor::TORQUE;
  wml_->targetTorque() = u_.T_lwl;
  wmr_->targetTorque() = u_.T_lwr;
  jml1_->targetTorque() = u_.T_jl1;
  jml2_->targetTorque() = u_.T_jl2;
  jmr1_->targetTorque() = u_.T_jr1;
  jmr2_->targetTorque() = u_.T_jr2;
}

// 运动控制器
void WBRChassis::locomotionController(void) {
  // 状态反馈矩阵K(ll,lr)
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 10; j++) {
      locomotion_controller_.K[i][j] =
          locomotion_controller_.KK1[i][j] +
          locomotion_controller_.KK2[i][j] * fdb_.ll +
          locomotion_controller_.KK3[i][j] * fdb_.lr;
    }
  }

  // u = K(xd-x)
  for (int i = 0; i < 4; i++) {
    float ui = 0;
    for (int j = 0; j < 10; j++) {
      ui += locomotion_controller_.K[i][j] * (ref_.xd[j] - fdb_.x[j]);
    }
    locomotion_controller_.u[i] = ui;
  }

  // 模型自适应力矩补偿
  if (jump_controller_.stage == JumpController_t::JumpStage_e::FINISH) {
    // 正常行驶
    locomotion_controller_.model_adapt.T_lwl =
        locomotion_controller_.model_adapt.k *
        (predictor_.theta_wl_D1_hat - sensor_.theta_wl_D1);
    locomotion_controller_.model_adapt.T_lwr =
        locomotion_controller_.model_adapt.k *
        (predictor_.theta_wr_D1_hat - sensor_.theta_wr_D1);
  } else {
    // 跳跃中轮离地降速
  }

  // 腿部力矩：运动控制器 + 自适应
  u_.T_lwl = locomotion_controller_.u[0] +
             locomotion_controller_.model_adapt.T_lwl;
  u_.T_lwr = locomotion_controller_.u[1] +
             locomotion_controller_.model_adapt.T_lwr;
  u_.T_bll = locomotion_controller_.u[2];
  u_.T_blr = locomotion_controller_.u[3];
}

// 腿部控制器
void WBRChassis::legController(void) {
  // TODO 腿部控制器设计

  u_.F_bll =;
  u_.F_blr =;
}

// 关节控制器
void WBRChassis::jointController(void) {
  // 腿部关节力矩映射
  u_.T_jl1 =
      observer_.Jl.JT[0][0] * u_.F_bll + observer_.Jl.JT[0][1] * u_.T_bll;
  u_.T_jl2 =
      observer_.Jl.JT[1][0] * u_.F_bll + observer_.Jl.JT[1][1] * u_.T_bll;
  u_.T_jr1 =
      observer_.Jr.JT[0][0] * u_.F_blr + observer_.Jr.JT[0][1] * u_.T_blr;
  u_.T_jr2 =
      observer_.Jr.JT[1][0] * u_.F_blr + observer_.Jr.JT[1][1] * u_.T_blr;
}

// 获取传感器数据(单位: rad, rad/s)
void WBRChassis::sensor(void) {
  sensor_.theta_wl = math::deg2rad(wml_->realAngle());
  sensor_.theta_wl_D1 = math::dps2radps(wml_->realSpeed());
  sensor_.theta_wr = math::deg2rad(wmr_->realAngle());
  sensor_.theta_wr_D1 = math::dps2radps(wmr_->realSpeed());
  const float joint_angle_l1 =
      param_.joint_reference_angle_deg +
      (jml1_->motor_data_.ecd_angle - param_.joint_encoder_offset_deg[0]) /
      jml1_->ratio_;
  const float joint_angle_l2 =
      param_.joint_reference_angle_deg +
      (jml2_->motor_data_.ecd_angle - param_.joint_encoder_offset_deg[1]) /
      jml2_->ratio_;
  const float joint_angle_r1 =
      param_.joint_reference_angle_deg +
      (jmr1_->motor_data_.ecd_angle - param_.joint_encoder_offset_deg[2]) /
      jmr1_->ratio_;
  const float joint_angle_r2 =
      param_.joint_reference_angle_deg +
      (jmr2_->motor_data_.ecd_angle - param_.joint_encoder_offset_deg[3]) /
      jmr2_->ratio_;
  sensor_.theta_bjl1 = math::deg2rad(math::loopLimit(joint_angle_l1, 0.f, 360.f));
  sensor_.theta_bjl1_D1 = math::dps2radps(jml1_->realSpeed());
  sensor_.theta_bjl2 = math::deg2rad(math::loopLimit(joint_angle_l2, 0.f, 360.f));
  sensor_.theta_bjl2_D1 = math::dps2radps(jml2_->realSpeed());
  sensor_.theta_bjr1 = math::deg2rad(math::loopLimit(joint_angle_r1, 0.f, 360.f));
  sensor_.theta_bjr1_D1 = math::dps2radps(jmr1_->realSpeed());
  sensor_.theta_bjr2 = math::deg2rad(math::loopLimit(joint_angle_r2, 0.f, 360.f));
  sensor_.theta_bjr2_D1 = math::dps2radps(jmr2_->realSpeed());
  sensor_.theta_b = imu_->euler_rad_.pitch - param_.pitch_bias;
  sensor_.theta_b_D1 = imu_->gyro_sensor_[1];
  sensor_.phi += math::loopLimit(imu_->euler_rad_.yaw - sensor_.phi, -PI, PI);
  sensor_.phi_D1 = imu_->gyro_sensor_[2];
  sensor_.psi = imu_->euler_rad_.roll - param_.roll_bias;
  sensor_.psi_D1 = imu_->gyro_sensor_[0];
  if (mode_ == WBRChassis::STOP) {
    tof_.acquire_lap = 5;
  }
}

// 观测器
void WBRChassis::observer(void) {
  // 腿部状态
  LegState::q_t qll, qlr;
  qll.phi1 = sensor_.theta_bjl1;
  qll.phi2 = sensor_.theta_bjl2;
  qlr.phi1 = sensor_.theta_bjr1;
  qlr.phi2 = sensor_.theta_bjr2;
  observer_.ll =
      leg_model_.legFkine(qll, param_.l_1a, param_.l_1u, param_.l_2a,
                          param_.l_2u, leg_model_.k_xe, leg_model_.k_ye);
  observer_.lr =
      leg_model_.legFkine(qlr, param_.l_1a, param_.l_1u, param_.l_2a,
                          param_.l_2u, leg_model_.k_xe, leg_model_.k_ye);
  observer_.Jl = leg_model_.legJacob(observer_.ll, param_.l_1u, param_.l_2u);
  observer_.Jr = leg_model_.legJacob(observer_.lr, param_.l_1u, param_.l_2u);
  observer_.ll.p_D1.l_D1 = observer_.Jl.J[0][0] * sensor_.theta_bjl1_D1 +
                           observer_.Jl.J[0][1] * sensor_.theta_bjl2_D1;
  observer_.ll.p_D1.theta_D1 = observer_.Jl.J[1][0] * sensor_.theta_bjl1_D1 +
                               observer_.Jl.J[1][1] * sensor_.theta_bjl2_D1;
  observer_.lr.p_D1.l_D1 = observer_.Jr.J[0][0] * sensor_.theta_bjr1_D1 +
                           observer_.Jr.J[0][1] * sensor_.theta_bjr2_D1;
  observer_.lr.p_D1.theta_D1 = observer_.Jr.J[1][0] * sensor_.theta_bjr1_D1 +
                               observer_.Jr.J[1][1] * sensor_.theta_bjr2_D1;

  // 地面支持力估计
  observer_.ll_D2 = observer_.ll_D2_filter.update(observer_.ll.p_D1.l_D1 -
                                                  observer_.last_ll_D1);
  observer_.lr_D2 = observer_.lr_D2_filter.update(observer_.lr.p_D1.l_D1 -
                                                  observer_.last_lr_D1);
  observer_.last_ll_D1 = observer_.ll.p_D1.l_D1;
  observer_.last_lr_D1 = observer_.lr.p_D1.l_D1;

  // 反馈状态
  fdb_.s = 0.5f * param_.R_w * (sensor_.theta_wl + sensor_.theta_wr);
  fdb_.s_D1 = 0.5f * param_.R_w * (sensor_.theta_wl_D1 + sensor_.theta_wr_D1);
  fdb_.phi = sensor_.phi;
  fdb_.phi_D1 = sensor_.phi_D1;
  fdb_.theta_ll = sensor_.theta_b + observer_.ll.p.theta;
  fdb_.theta_ll_D1 = sensor_.theta_b_D1 + observer_.ll.p_D1.theta_D1;
  fdb_.theta_lr = sensor_.theta_b + observer_.lr.p.theta;
  fdb_.theta_lr_D1 = sensor_.theta_b_D1 + observer_.lr.p_D1.theta_D1;
  fdb_.theta_b = sensor_.theta_b;
  fdb_.theta_b_D1 = sensor_.theta_b_D1;
  fdb_.psi = sensor_.psi;
  fdb_.psi_D1 = sensor_.psi_D1;
  fdb_.ll = observer_.ll.p.l;
  fdb_.lr = observer_.lr.p.l;
  fdb_.l = 0.5f * (fdb_.ll + fdb_.lr);

  // 状态向量
  fdb_.x[0] = fdb_.s;
  fdb_.x[1] = fdb_.s_D1;
  fdb_.x[2] = fdb_.phi;
  fdb_.x[3] = fdb_.phi_D1;
  fdb_.x[4] = fdb_.theta_ll;
  fdb_.x[5] = fdb_.theta_ll_D1;
  fdb_.x[6] = fdb_.theta_lr;
  fdb_.x[7] = fdb_.theta_lr_D1;
  fdb_.x[8] = fdb_.theta_b;
  fdb_.x[9] = fdb_.theta_b_D1;
}

// 预测器
void WBRChassis::predictor(void) {
  // 上一周期预测误差
  for (int i = 0; i < 10; i++) {
    predictor_.error[i] = predictor_.xhat[i] - fdb_.x[i];
  }
  // 状态空间模型
  float ll = fdb_.ll;
  float lr = fdb_.lr;
  float ll2 = fdb_.ll * fdb_.ll;
  float lr2 = fdb_.lr * fdb_.lr;
  float lllr = fdb_.ll * fdb_.lr;
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 3; j++) {
      model_.A[i * 2 + 1][j * 2 + 4] =
          model_.KA1[i][j] + model_.KA2[i][j] * ll + model_.KA3[i][j] * ll2 +
          model_.KA4[i][j] * lr + model_.KA5[i][j] * lr2 +
          model_.KA6[i][j] * lllr;
    }
    for (int k = 0; k < 4; k++) {
      model_.B[i * 2 + 1][k] = model_.KB1[i][k] + model_.KB2[i][k] * ll +
                               model_.KB3[i][k] * ll2 + model_.KB4[i][k] * lr +
                               model_.KB5[i][k] * lr2 + model_.KB6[i][k] * lllr;
    }
  }
  // 状态预测
  // for (int i = 0; i < 10; i++) {
  //   float xhat_i = fdb_.x[i];
  //   for (int j = 0; j < 10; j++) {
  //     xhat_i += model_.A[i][j] * fdb_.x[j] * 1e-3f;
  //   }
  //   for (int k = 0; k < 4; k++) {
  //     xhat_i += model_.B[i][k] * locomotion_controller_.u[k] * 1e-3f;
  //   }
  //   predictor_.xhat[i] = xhat_i;
  // }
  for (int i = 1; i <= 3; i += 2) {
    // 仅预测轮速，降低计算消耗
    float xhat_i = fdb_.x[i];
    for (int j = 0; j < 10; j++) {
      xhat_i += model_.A[i][j] * fdb_.x[j] * 1e-3f;
    }
    for (int k = 0; k < 4; k++) {
      xhat_i += model_.B[i][k] * locomotion_controller_.u[k] * 1e-3f;
    }
    predictor_.xhat[i] = xhat_i;
  }
  // 轮速预测
  predictor_.s_D1_hat = predictor_.xhat[1];
  predictor_.phi_D1_hat = predictor_.xhat[3];
  predictor_.theta_wl_D1_hat =
      (predictor_.s_D1_hat - predictor_.phi_D1_hat * param_.R_l) / param_.R_w;
  predictor_.theta_wr_D1_hat =
      (predictor_.s_D1_hat + predictor_.phi_D1_hat * param_.R_l) / param_.R_w;
}

// 处理目标状态
void WBRChassis::handleRef(void) {
  // 目标状态处理
  // TODO reference处理（限幅，限变化幅度）

  ref_.xd[0] = ref_.sd;
  ref_.xd[1] = ref_.sd_D1;
  ref_.xd[2] = ref_.phid;
  ref_.xd[3] = ref_.phid_D1;
}

void WBRChassis::crawlController(void) {
  // 占位，非必做项
}

// 停止状态控制
void WBRChassis::stopController(void) {
  // 重置传感器状态
  tof_.acquire_lap = 5;

  wml_->method_ = Motor::SPEED;
  wmr_->method_ = Motor::SPEED;
  jml1_->method_ = Motor::TORQUE;
  jml2_->method_ = Motor::TORQUE;
  jmr1_->method_ = Motor::TORQUE;
  jmr2_->method_ = Motor::TORQUE;

  wml_->targetTorque() = 0;
  wmr_->targetTorque() = 0;
  jml1_->targetTorque() = 0;
  jml2_->targetTorque() = 0;
  jmr1_->targetTorque() = 0;
  jmr2_->targetTorque() = 0;

  // 重置目标状态
  ref_.sd = fdb_.s;
  ref_.sd_D1 = 0;
  ref_.phid = fdb_.phi;
  ref_.phid_D1 = 0;
  ref_.l = 0.12f;
}

// 跳跃控制
void WBRChassis::jumpController(void) {
  // 非必做项
  if (jump_controller_.stage == JumpController_t::JumpStage_e::FINISH) {
    return;
  }
  // 开始阶段
  else if (jump_controller_.stage == JumpController_t::JumpStage_e::START) {
  }
  // 下蹲阶段
  else if (jump_controller_.stage == JumpController_t::JumpStage_e::SQUAT) {
  }
  // 起跳阶段
  else if (jump_controller_.stage == JumpController_t::JumpStage_e::JUMP) {
  }
  // 收腿阶段
  else if (jump_controller_.stage == JumpController_t::JumpStage_e::RECOVERY) {
  }
  // 伸腿缓冲
  else if (jump_controller_.stage == JumpController_t::JumpStage_e::STRETCH) {
  }

  // 腿部支持力(覆盖legController中的腿部支持力)
  // u_.F_bll =;
  // u_.F_blr =;
}

WBRChassis::LegState_t WBRChassis::LegModel_t::legFkine(
    LegState::q_t q, const float& l_1a, const float& l_1u, const float& l_2a,
    const float& l_2u, const float k_xe[6], const float k_ye[6]) {
  WBRChassis::LegState_t state;

  // TODO 腿部正运动学解算

  return state;
}

// 腿部逆运动学
WBRChassis::LegState::q_t WBRChassis::LegModel_t::legIkine(
    LegState::p_t p, const float& l_1a, const float& l_1u, const float& l_1d,
    const float& l_2a, const float& l_2u, const float& l_2d) {
  WBRChassis::LegState::q_t q;

  float l = p.l;
  float theta = p.theta;

  float l_1 = sqrtf(l_1a * l_1a + l * l + 2.f * l_1a * l * sinf(theta));
  float l_2 = sqrtf(l_2a * l_2a + l * l - 2.f * l_2a * l * sinf(theta));
  q.phi1 = acosf((l_1 * l_1 + l_1a * l_1a - l * l) / (2.f * l_1 * l_1a)) +
           acosf((l_1 * l_1 + l_1u * l_1u - l_1d * l_1d) / (2.f * l_1 * l_1u));
  q.phi2 = acosf((l_2 * l_2 + l_2a * l_2a - l * l) / (2.f * l_2 * l_2a)) +
           acosf((l_2 * l_2 + l_2u * l_2u - l_2d * l_2d) / (2.f * l_2 * l_2u));

  return q;
}

// 腿部雅可比矩阵
WBRChassis::LegJacob_t WBRChassis::LegModel_t::legJacob(LegState_t state,
                                                        const float& l_1u,
                                                        const float& l_2u) {
  WBRChassis::LegJacob_t jacob;

  float phi1 = state.q.phi1;
  float phi2 = state.q.phi2;
  float l = state.p.l;
  float theta = state.p.theta;
  float xe = state.xe;
  float ye = state.ye;
  float x1 = state.x1;
  float y1 = state.y1;
  float x2 = state.x2;
  float y2 = state.y2;

  // TODO 雅可比矩阵计算

  jacob.invJ[0][0] =;
  jacob.invJ[0][1] =;
  jacob.invJ[1][0] =;
  jacob.invJ[1][1] =;

  jacob.J[0][0] =;
  jacob.J[0][1] =;
  jacob.J[1][0] =;
  jacob.J[1][1] =;

  jacob.JT[0][0] = jacob.J[0][0];
  jacob.JT[0][1] = jacob.J[1][0];
  jacob.JT[1][0] = jacob.J[0][1];
  jacob.JT[1][1] = jacob.J[1][1];

  return jacob;
}

// TOF处理
void WBRChassis::Tof_t::handle(void) {
  if (HAL_GetTick() % acquire_lap == 0) {
    if (TOF_IIC_Update_Distance(&tof_data)) {
      tof_connect.refresh();
      acquire_lap = 5;
    }
  }
  if (!tof_connect.check()) {
    tof_data.dis = 10;
    tof_data.dis_status = 0;
    acquire_lap = 6000e3; // 相当于不再读取
  }
}
