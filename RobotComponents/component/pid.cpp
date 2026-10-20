/**
 *******************************************************************************
 * @file      :pid.cpp
 * @brief     :位置式 PID 控制器实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-20      lishaojun       1. 初版：积分限幅/抗饱和/微分滤波
 *******************************************************************************
 * @attention :
 *  1. calc() 内部顺序：微分 → 比例 → 抗饱和判断 → 积分 → 输出限幅。
 *     抗饱和判断必须在积分之前，否则积分仍会累积。
 *  2. 微分项对"测量值"求导（不是误差），避免目标跳变产生冲击。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#include "pid.hpp"

#include "base.hpp"   // Bound()

namespace hello_world {
namespace pid {

// ---------------- ③ 构造函数 ----------------
Pid::Pid(float kp, float ki, float kd, float out_lim, float i_lim)
    : kp_(kp), ki_(ki), kd_(kd), out_lim_(out_lim), i_lim_(i_lim) {}

// ---------------- ④ 成员函数 ----------------

void Pid::init(float kp, float ki, float kd, float out_lim, float i_lim) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  out_lim_ = out_lim;
  i_lim_ = i_lim;
  reset();                 // 重新初始化时清空内部状态
}

void Pid::setGains(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void Pid::setOutLim(float out_lim) {
  out_lim_ = (out_lim > 0.0f) ? out_lim : 0.0f;
}

void Pid::setILim(float i_lim) {
  i_lim_ = (i_lim > 0.0f) ? i_lim : 0.0f;
}

void Pid::setDFilterAlpha(float alpha) {
  // alpha 必须在 (0, 1]，否则滤波会死锁或失效
  if (alpha <= 0.0f) alpha = 0.01f;
  if (alpha > 1.0f)  alpha = 1.0f;
  d_alpha_ = alpha;
}

// ---------------- 核心 ----------------
float Pid::calc(float target, float measure, float dt) {
  // dt 非法直接返回 0，避免除零
  if (dt <= 0.0f) {
    return 0.0f;
  }

  const float e = target - measure;

  // ---------- 微分：对测量 + 低通 ----------
  float d_raw = 0.0f;
  if (!is_first_) {
    d_raw = -kd_ * (measure - last_measure_) / dt;
  }
  d_filt_ = d_alpha_ * d_raw + (1.0f - d_alpha_) * d_filt_;

  // ---------- 比例 ----------
  const float p_out = kp_ * e;

  // ---------- 抗饱和判断 ----------
  const float u_unsat = p_out + i_out_ + d_filt_;
  const bool pushing_high = (u_unsat >  out_lim_) && (e > 0.0f);
  const bool pushing_low  = (u_unsat < -out_lim_) && (e < 0.0f);

  // ---------- 积分 + 限幅 ----------
  if (!pushing_high && !pushing_low) {
    i_out_ += ki_ * e * dt;
    i_out_ = Bound(i_out_, -i_lim_, i_lim_);
  }

  // ---------- 输出 + 限幅 ----------
  float u = p_out + i_out_ + d_filt_;
  u = Bound(u, -out_lim_, out_lim_);

  last_measure_ = measure;
  is_first_ = false;
  return u;
}

void Pid::reset() {
  i_out_ = 0.0f;
  d_filt_ = 0.0f;
  last_measure_ = 0.0f;
  is_first_ = true;
}

}  // namespace pid
}  // namespace hello_world