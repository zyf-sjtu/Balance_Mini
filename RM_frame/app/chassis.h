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

#ifndef CHASSIS_H
#define CHASSIS_H

#include "base/common/filter.h"
#include "base/common/math.h"
#include "base/common/pid.h"
#include "base/imu/imu.h"
#include "base/motor/motor.h"
#include "base/tof/TOFSenseF_IIC.h"

// 平衡底盘类(双足轮腿机器人WBR)
// 单位: 长度-m, 角度-rad, 角速度-rad/s
class WBRChassis {
public:
  // 工作状态
  typedef enum WorkState {
    STOP, // 停止
    DIRECT, // 直接控制底盘
    GYRO, // 陀螺
    CRAWL, // 爬行(仅控制轮电机速度)
  } Mode_e;

  // 腿部状态量
  typedef struct LegState {
    struct q_t {
      float phi1, phi2;
    } q;

    struct p_t {
      float l, theta;
    } p;

    struct p_D1_t {
      float l_D1, theta_D1;
    } p_D1;

    float xe, ye;
    float x1, y1, x2, y2;
  } LegState_t;

  // 腿部雅可比矩阵
  typedef struct LegJacob {
    float J[2][2]; // 雅可比矩阵
    float JT[2][2]; // 转置
    float invJ[2][2]; // 逆
  } LegJacob_t;

  // 跳跃指令(开始/中止)
  typedef enum JumpCmd {
    START,
    ABORT,
  } JumpCmd_e;

public:
  WBRChassis(Motor* wml, Motor* wmr, Motor* jml1, Motor* jml2, Motor* jmr1,
             Motor* jmr2, IMU* chassis_imu);

  // 底盘初始化
  void init(void);

  // 底盘控制处理
  void handle(void);

  // 跳跃指令
  void setJump(WBRChassis::JumpCmd_e cmd);

  // 目标状态变量引用
  float& targetS_D1(void) { return ref_.sd_D1; }
  float& targetPhi(void) { return ref_.phid; }
  float& targetL(void) { return ref_.l; }

  float& targetVx(void) { return ref_.vx; }
  float& targetVy(void) { return ref_.vy; }
  float& targetWz(void) { return ref_.wz; }

private:
  // WBR控制器
  void WBRController(void);

  // 运动控制器
  void locomotionController(void);

  // 腿部控制器
  void legController(void);

  // 关节控制器
  void jointController(void);

  // 获取传感器数据
  void sensor(void);

  // 观测器
  void observer(void);

  // 预测器
  void predictor(void);

  // 处理目标状态
  void handleRef(void);

  // 爬行控制器
  void crawlController(void);

  // 停止状态控制
  void stopController(void);

  // 跳跃控制
  void jumpController(void);

public:
  // 电机指针
  Motor *wml_, *wmr_;
  Motor *jml1_, *jml2_, *jmr1_, *jmr2_;
  // imu指针
  IMU* imu_;

  // 工作状态
  Mode_e mode_;

  // 目标状态
  struct Ref_t {
    float xd[10]; // 状态向量
    float sd, sd_D1; // 位移/速度(xd[0], xd[1])
    float phid, phid_D1; // yaw/wz(xd[2], xd[3])
    float psid, psid_D1; // roll
    float l, l_extra; // l-目标腿长 / l_extra-额外腿长（保持平衡）
    float ld; // ld-目标腿长（控制输入）

    float vx, vy, wz; // 全向移动目标速度(FOLLOW,GYRO)
    float delta_phi, extra_phi; // 跟随/附加phi角度(rad)

    float sd_D1_max = 0.8f;
    float wz_max = 3.f * PI;
  } ref_;

  // 反馈状态
  struct Fdb_t {
    float x[10]; // 状态向量
    float s, s_D1; // 位移/速度(x[0], x[1])
    float phi, phi_D1; // yaw/wz(x[2], x[3])
    // 腿部/机体θ/wy(x[4]~x[9])
    float theta_ll, theta_ll_D1;
    float theta_lr, theta_lr_D1;
    float theta_b, theta_b_D1;
    float psi, psi_D1; // roll
    float ll, lr, l; // 腿长
  } fdb_;

  // 控制量状态(力/力矩)
  struct U_t {
    float T_lwl, T_lwr; // 驱动轮力矩
    float T_bll, T_blr; // 腿部等效力矩
    float F_bll, F_blr; // 腿部等效支持力
    float T_jl1, T_jl2, T_jr1, T_jr2; // 腿部关节力矩
  } u_;

  // 传感器数据
  struct Sensor_t {
    // 轮电机角度/角速度
    float theta_wl, theta_wl_D1;
    float theta_wr, theta_wr_D1;
    // 关节电机角度/角速度
    float theta_bjl1, theta_bjl1_D1;
    float theta_bjl2, theta_bjl2_D1;
    float theta_bjr1, theta_bjr1_D1;
    float theta_bjr2, theta_bjr2_D1;
    // 机体俯仰(pitch)角度/角速度
    float theta_b, theta_b_D1;
    // yaw角度/角速度
    float phi, phi_D1;
    // roll角度/角速度
    float psi, psi_D1;
  } sensor_;

  // 机器人参数
  struct Param_t {
    float R_w = 35e-3f; // 轮半径-m
    float R_l = 153.3e-3f; // 1/2轮距-m
    // 质量参数-kg
    float m_w = 0.08f;
    float m_l = 0.51f;
    float m_b = 2.319f;
    // 腿部长度参数-m
    float l_1a = 0.075f / 2, l_1u = 0.075, l_1d = 0.135;
    float l_2a = 0.075f / 2, l_2u = 0.075, l_2d = 0.135;//计算得最大腿长为0.2066
    // 陀螺仪角度偏置
    float pitch_bias = 0;
    float roll_bias = 0;

    float joint_encoder_offset_deg[4] = {
        -39.9567146f, -56.4140282f, 11.3706808f, 27.4105339f
    };
    const float joint_reference_angle_deg = 210.f;
  } param_;

  // 状态空间模型
  struct Model_t {
    // A矩阵多项式拟合系数(i=2,4,6,8,10;j=5,7,9)
    float KA1[5][3] = {
        -6.27, -6.27, -0.00,
        -2.08, 2.08, -0.00,
        152.62, -12.42, -0.00,
        -12.42, 152.62, -0.00,
        -1.29, -1.29, 4.33,
    };
    float KA2[5][3] = {
        -18.51, 3.41, 0.00,
        -7.68, 1.40, 0.00,
        -499.12, 38.01, 0.00,
        -14.39, 3.06, 0.00,
        -5.76, 0.95, 0.00,
    };
    float KA3[5][3] = {
        18.29, -3.80, -0.00,
        7.85, -1.61, -0.00,
        574.56, -42.33, -0.00,
        10.53, -2.67, -0.00,
        5.86, -1.05, -0.00,
    };
    float KA4[5][3] = {
        3.41, -18.51, 0.00,
        -1.40, 7.68, 0.00,
        3.06, -14.39, 0.00,
        38.01, -499.12, 0.00,
        0.95, -5.76, 0.00,
    };
    float KA5[5][3] = {
        -3.80, 18.29, 0.00,
        1.61, -7.85, 0.00,
        -2.67, 10.53, 0.00,
        -42.33, 574.56, 0.00,
        -1.05, 5.86, -0.00,
    };
    float KA6[5][3] = {
        0.85, 0.85, -0.00,
        -0.55, 0.55, -0.00,
        -1.83, 12.28, -0.00,
        12.28, -1.83, -0.00,
        0.24, 0.24, -0.00,
    };
    // B矩阵多项式拟合系数(i=2,4,6,8,10;j=1,2,3,4)
    float KB1[5][4] = {
        28.92, 28.92, -8.15, -8.15,
        -122.08, 122.08, -3.08, 3.08,
        -350.05, -27.54, 126.20, -11.27,
        -27.54, -350.05, -11.27, 126.20,
        0.31, 0.31, -231.70, -231.70,
    };
    float KB2[5][4] = {
        -6.69, 7.09, 25.12, 1.81,
        -2.27, 2.90, 10.01, 0.74,
        1385.31, 79.04, -615.48, 20.35,
        -16.50, 6.32, 28.85, 1.59,
        1.19, 1.97, 5.12, 0.50,
    };
    float KB3[5][4] = {
        10.61, -7.46, -27.93, -1.37,
        3.99, -3.16, -11.53, -0.58,
        -1732.72, -83.07, 824.73, -15.22,
        19.75, -5.28, -27.34, -1.02,
        -0.92, -2.07, -5.38, -0.38,
    };
    float KB4[5][4] = {
        7.09, -6.69, 1.81, 25.12,
        -2.90, 2.27, -0.74, -10.01,
        6.32, -16.50, 1.59, 28.85,
        79.04, 1385.31, 20.35, -615.48,
        1.97, 1.19, 0.50, 5.12,
    };
    float KB5[5][4] = {
        -7.46, 10.61, -1.37, -27.93,
        3.16, -3.99, 0.58, 11.53,
        -5.28, 19.75, -1.02, -27.34,
        -83.07, -1732.72, -15.22, 824.73,
        -2.07, -0.92, -0.38, -5.38,
    };
    float KB6[5][4] = {
        0.27, 0.27, -1.80, -1.80,
        -0.49, 0.49, 0.69, -0.69,
        -4.60, 7.91, -2.23, -19.91,
        7.91, -4.60, -19.91, -2.23,
        0.07, 0.07, -0.50, -0.50,
    };

    float A[10][10] = {
        0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };
    float B[10][4] = {0};
  } model_;

  // 腿部模型
  struct LegModel_t {
    // 腿部正运动学多项式拟合参数
    float k_xe[6] = {0, -0.1481f, 0.1481f, 0.0194f, -0.0194f, 0};
    float k_ye[6] = {0.5824f, -0.1240f, -0.1240, 3.78e-3f, 3.78e-3f, 2.01e-2f};
    // 腿部正运动学
    LegState_t legFkine(LegState::q_t q, const float& l_1a, const float& l_1u,
                        const float& l_2a, const float& l_2u,
                        const float k_xe[6], const float k_ye[6]);
    // 腿部逆运动学
    LegState::q_t legIkine(LegState::p_t p, const float& l_1a,
                           const float& l_1u, const float& l_1d,
                           const float& l_2a, const float& l_2u,
                           const float& l_2d);
    // 腿部雅可比矩阵
    LegJacob_t legJacob(LegState_t state, const float& l_1u, const float& l_2u);
  } leg_model_;

  // 运动控制器
  struct LocomotionController_t {
    // TODO K矩阵的拟合系数 K00 K01 K10 K02 K11 K20

    // 状态反馈矩阵K
    float K[4][10];
    float KK1[4][10] = {
      -0.19,-0.57,-0.26,-0.07,-0.99,-0.04,-0.62,-0.02,-0.56,-0.05,
      -0.19,-0.57,0.26,0.07,-0.62,-0.02,-0.99,-0.04,-0.56,-0.05,
      0.42,1.27,-0.49,-0.11,5.59,0.39,-1.72,0.05,-9.58,-0.34,
      0.42,1.27,0.49,0.11,-1.72,0.05,5.59,0.39,-9.58,-0.34,
      };
    float KK2[4][10] = {
      -0.25,-0.82,0.39,0.15,-6.96,-1.46,2.67,0.07,1.43,0.09,
      0.21,0.59,0.32,0.13,-5.88,-1.15,3.00,0.30,0.81,0.08,
      -1.42,-4.08,-0.72,-0.30,-11.85,0.17,-18.24,-1.85,-9.57,-0.57,
      -0.27,-1.02,0.49,0.16,6.76,0.63,10.28,0.38,7.31,0.39,
      };
    float KK3[4][10] = {
      0.30,0.87,-0.36,-0.12,3.52,-0.20,-1.74,0.05,-1.43,-0.07,
      -0.19,-0.59,-0.35,-0.13,3.89,0.10,-3.20,-0.46,-0.61,-0.06,
      1.24,3.64,0.99,0.44,8.20,-0.30,26.15,3.10,12.52,0.68,
      0.36,1.22,-0.69,-0.27,1.12,0.49,-15.76,-0.93,-9.86,-0.50,
      };
    float KK4[4][10] = {
      0.21,0.59,-0.32,-0.13,3.00,0.30,-5.88,-1.15,0.81,0.08,
      -0.25,-0.82,-0.39,-0.15,2.67,0.07,-6.96,-1.46,1.43,0.09,
      -0.27,-1.02,-0.49,-0.16,10.28,0.38,6.76,0.63,7.31,0.39,
      -1.42,-4.08,0.72,0.30,-18.24,-1.85,-11.85,0.17,-9.57,-0.57,
      };
    float KK5[4][10] = {
      -0.19,-0.59,0.35,0.13,-3.20,-0.46,3.89,0.10,-0.61,-0.06,
      0.30,0.87,0.36,0.12,-1.74,0.05,3.52,-0.20,-1.43,-0.07,
      0.36,1.22,0.69,0.27,-15.76,-0.93,1.12,0.49,-9.86,-0.50,
      1.24,3.64,-0.99,-0.44,26.15,3.10,8.20,-0.30,12.52,0.68,
      };
    float KK6[4][10] = {
      -0.04,-0.11,-0.07,-0.03,2.08,0.65,-0.21,-0.09,-0.77,-0.08,
      -0.04,-0.11,0.07,0.03,-0.21,-0.09,2.08,0.65,-0.77,-0.08,
      0.52,1.55,-0.24,-0.37,18.90,2.91,-22.69,-4.43,0.51,0.07,
      0.52,1.55,0.24,0.37,-22.69,-4.43,18.90,2.91,0.51,0.07,
      };
    // 控制向量u
    float u[4];

    // 模型自适应力矩补偿
    struct ModelAdapt_t {
      float k = 0.2;
      float T_lwl, T_lwr;
      float k_w_D1; // 离地状态目标速度比例
    } model_adapt;
  } locomotion_controller_;

  // 腿部控制器
  struct LegController_t {
    PID psi_ppid = PID(4.2, 0, 2, 0, 4, 0.25); // roll position pid
    PID psi_spid = PID(3.6, 0, 0, 0, 5, 1); // roll speed pid
    PID l_pid = PID(200, 0, 2500, 0, 10, 0.2); // 平均腿长pid

    float eta_l = 0.22f; // 腿部质量比例(根据质心位置函数和功能关系得到)

    float F_psi, F_l, F_g, F_i; // 腿部支持力分量
    float k_F_psi; // 维持roll的力分量比例(弱化长短腿问题)
  } leg_controller_;

  // 跳跃控制器
  struct JumpController_t {
    uint32_t tick;

    enum JumpStage_e {
      FINISH,
      START,
      SQUAT,
      JUMP,
      RECOVERY,
      HOLD,
      STRETCH,
    } stage;

    bool prepare_stair = false;
    PID l_pid_squat = PID(1e3, 0, 1.5e5, 0, 80);
    PID l_pid_jump = PID(2e4, 0, 5e4, 0, 1000);
    PID l_pid_recovery = PID(1.8e3, 0, 2e5, 0, 160);
    float F_l; // leg support force(N)
  } jump_controller_;

  // 观测器(传感器数据->状态量)
  struct Observer {
    // 腿部状态量
    LegState ll, lr;
    // 腿部雅可比矩阵
    LegJacob_t Jl, Jr;

    // 腿长一阶导数记录
    float last_ll_D1, last_lr_D1;
    // 腿长二阶导数
    float ll_D2, lr_D2;
    // 腿长二阶导数滤波
    LowPassFilter ll_D2_filter = LowPassFilter(0.05f);
    LowPassFilter lr_D2_filter = LowPassFilter(0.05f);
  } observer_;

  // 预测器
  struct Predictor_t {
    float xhat[10];
    float error[10];
    float s_D1_hat, phi_D1_hat;
    float theta_wl_D1_hat, theta_wr_D1_hat;
  } predictor_;

  // 爬行控制器
  struct CrawlController_t {
    float crawl_joint_damping = 3.6f;
    float jl1_u, jl2_u, jr1_u, jr2_u;
    float wml_u, wmr_u;
    float wml_follow_speed, wmr_follow_speed;
    PID wml_spid = PID(0.5, 0, 0, 0, 10000);
    PID wmr_spid = PID(0.5, 0, 0, 0, 10000);
  } crawl_controller_;

  struct Tof_t {
    tofsense_f_output_t tof_data;
    Connect tof_connect = Connect(500);
    uint32_t acquire_lap = 5;
    void handle(void);
  } tof_;
};

#endif  // CHASSIS_H
