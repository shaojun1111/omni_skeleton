/**
 *******************************************************************************
 * @file      :pid.hpp
 * @brief     :位置式 PID 控制器（积分限幅 / 抗饱和 / 输出限幅 / 微分滤波）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-20      lishaojun       1. 初版
 *******************************************************************************
 * @attention :
 *  1. 使用前必须先调用 init() 设置参数。
 *  2. dt 单位为秒（本项目 0.001f），传错会导致积分/微分异常。
 *  3. 切换模式、急停恢复、电机重新使能时，必须调用 reset()。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#ifndef ROBOT_COMPONENTS_PID_HPP_
#define ROBOT_COMPONENTS_PID_HPP_

namespace hello_world {
namespace pid {

class Pid {
 public:
  Pid() = default;
  Pid(float kp, float ki, float kd, float out_lim, float i_lim);

  /// @brief 初始化/重设参数
  void init(float kp, float ki, float kd, float out_lim, float i_lim);

  void setGains(float kp, float ki, float kd);
  void setOutLim(float out_lim);
  void setILim(float i_lim);
  void setDFilterAlpha(float alpha);

  /// @brief 计算一次输出
  /// @param target  目标值（单位与 measure 一致）
  /// @param measure 反馈值
  /// @param dt      控制周期，单位 s（本项目 0.001f）
  /// @return 限幅后的输出
  float calc(float target, float measure, float dt);

  /// @brief 清空内部状态（积分/微分历史）。切换模式、急停、重新使能时调用
  void reset();

 private:
  float kp_ = 0.0f, ki_ = 0.0f, kd_ = 0.0f;
  float out_lim_ = 0.0f;   ///< 输出限幅（正数，对称限幅）
  float i_lim_ = 0.0f;     ///< 积分限幅（正数）
  float d_alpha_ = 0.2f;   ///< 微分低通系数 (0,1]

  float i_out_ = 0.0f;       ///< 积分累加量
  float d_filt_ = 0.0f;      ///< 滤波后的微分项
  float last_measure_ = 0.0f;
  bool  is_first_ = true;    ///< 第一次计算时没有"上一次测量"
};

}  // namespace pid
}  // namespace hello_world

#endif