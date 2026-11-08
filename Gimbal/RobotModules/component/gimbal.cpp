/**
 *******************************************************************************
 * @file      :gimbal.cpp
 * @brief     :云台运动模块实现（yaw / pitch 位置环）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-08      lishaojun       1. 角度差 → PID → MIT 力矩
 *******************************************************************************
 * @attention :
 *  1. yaw 的目标角先投影到反馈角所在的圈，再交给 PID，避免 ±π 处暴转。
 *  2. MIT 帧 kp = kd = 0，只填 torq，位置环完全由本文件的 PID 负责。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "gimbal.hpp"

#include "base.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
#include "motor.hpp"
#include "motor_DaMiao.hpp"
#include "pid.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace
{

constexpr float kDt = 0.001f;        ///< 控制周期，s
constexpr float kPitchMin = -0.7f;   ///< pitch 下限，rad（约 -40°）
constexpr float kPitchMax = 0.7f;    ///< pitch 上限，rad（约 +40°）

}  // namespace

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
namespace robot
{

void Gimbal::init(void)
{
  auto *yaw_motor = GetMotorYaw();
  auto *pitch_motor = GetMotorPitch();
  auto *yaw_pid = GetPidMotorYaw();
  auto *pitch_pid = GetPidMotorPitch();

  /* 目标角对齐当前反馈角：上电不跳 */
  yaw_fb_ = (yaw_motor != nullptr) ? yaw_motor->angle() : 0.0f;
  pitch_fb_ = (pitch_motor != nullptr) ? pitch_motor->angle() : 0.0f;
  target_.yaw = yaw_fb_;
  target_.pitch = hello_world::Bound(pitch_fb_, kPitchMin, kPitchMax);

  yaw_out_ = 0.0f;
  pitch_out_ = 0.0f;

  if (yaw_pid != nullptr) {
    yaw_pid->reset();
  }
  if (pitch_pid != nullptr) {
    pitch_pid->reset();
  }

  if (yaw_motor != nullptr) {
    static_cast<hello_world::motor::DaMiao *>(yaw_motor)
        ->setMitInput(yaw_fb_, 0.0f, 0.0f, 0.0f, 0.0f);
  }
  if (pitch_motor != nullptr) {
    static_cast<hello_world::motor::DaMiao *>(pitch_motor)
        ->setMitInput(pitch_fb_, 0.0f, 0.0f, 0.0f, 0.0f);
  }
}

void Gimbal::setTarget(const GimbalState& target)
{
  target_.yaw = target.yaw;
  target_.pitch = hello_world::Bound(target.pitch, kPitchMin, kPitchMax);
}

void Gimbal::update(void)
{
  auto *yaw_motor = GetMotorYaw();
  auto *pitch_motor = GetMotorPitch();
  auto *yaw_pid = GetPidMotorYaw();
  auto *pitch_pid = GetPidMotorPitch();

  if (yaw_motor == nullptr || pitch_motor == nullptr || yaw_pid == nullptr ||
      pitch_pid == nullptr) {
    return;
  }

  yaw_fb_ = yaw_motor->angle();
  pitch_fb_ = pitch_motor->angle();

  /* yaw：把目标投影到当前圈，误差落在 [-π, π) */
  const float yaw_target_near =
      yaw_fb_ + hello_world::AngleNormRad(target_.yaw - yaw_fb_);
  yaw_out_ = yaw_pid->calc(yaw_target_near, yaw_fb_, kDt);

  /* pitch：目标已在 setTarget 钳位，这里再钳一次防止直接写 target_ */
  const float pitch_target =
      hello_world::Bound(target_.pitch, kPitchMin, kPitchMax);
  pitch_out_ = pitch_pid->calc(pitch_target, pitch_fb_, kDt);

  /* MIT 帧：kp = kd = 0，只输出前馈力矩 */
  static_cast<hello_world::motor::DaMiao *>(yaw_motor)
      ->setMitInput(yaw_fb_, 0.0f, 0.0f, 0.0f, yaw_out_);
  static_cast<hello_world::motor::DaMiao *>(pitch_motor)
      ->setMitInput(pitch_fb_, 0.0f, 0.0f, 0.0f, pitch_out_);
}

void Gimbal::stop(void)
{
  auto *yaw_motor = GetMotorYaw();
  auto *pitch_motor = GetMotorPitch();
  auto *yaw_pid = GetPidMotorYaw();
  auto *pitch_pid = GetPidMotorPitch();

  yaw_out_ = 0.0f;
  pitch_out_ = 0.0f;

  if (yaw_pid != nullptr) {
    yaw_pid->reset();
  }
  if (pitch_pid != nullptr) {
    pitch_pid->reset();
  }

  /* 目标回到当前角，恢复控制时不跳 */
  if (yaw_motor != nullptr) {
    yaw_fb_ = yaw_motor->angle();
    target_.yaw = yaw_fb_;
    static_cast<hello_world::motor::DaMiao *>(yaw_motor)
        ->setMitInput(yaw_fb_, 0.0f, 0.0f, 0.0f, 0.0f);
  }

  if (pitch_motor != nullptr) {
    pitch_fb_ = pitch_motor->angle();
    target_.pitch = hello_world::Bound(pitch_fb_, kPitchMin, kPitchMax);
    static_cast<hello_world::motor::DaMiao *>(pitch_motor)
        ->setMitInput(pitch_fb_, 0.0f, 0.0f, 0.0f, 0.0f);
  }
}

}  // namespace robot

/* Private function definitions ----------------------------------------------*/
