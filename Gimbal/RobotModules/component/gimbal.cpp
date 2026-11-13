/**
 *******************************************************************************
 * @file      :gimbal.cpp
 * @brief     :云台运动模块实现（yaw / pitch 位置环）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-08      lishaojun       1. 角度差 → PID → MIT 力矩
 *  V1.1.0      2026-11-11      lishaojun       2. 遥控摇杆接管 yaw / pitch 目标
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
#include "imu.hpp"
#include "ins_imu.hpp"
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
constexpr float kYawStickGain = 2.5f;  ///< 左摇杆满杆 yaw 角速度，rad/s

/* yaw 关节角 → 世界 yaw 的比例 k 自标定参数。
 * 单周期增量小于 kCalMinStep 视为没动，大于 kCalMaxStep 视为跳变（丢帧/回绕）。 */
constexpr float kCalMinStep = 2e-4f;    ///< 0.2 rad/s 以下不采样
constexpr float kCalMaxStep = 0.05f;    ///< 单周期 0.05 rad 以上判跳变
constexpr float kCalMinTravel = 0.25f;  ///< 累计关节行程门槛，rad
constexpr float kCalMinR2 = 0.95f;      ///< 拟合决定系数门槛
constexpr float kCalMinScale = 0.1f;    ///< k 的合理下限（4 倍量程误差约 0.25）
constexpr float kCalMaxScale = 10.0f;   ///< k 的合理上限

/* δ 的固定偏置：实测前推时车往枪口右侧偏，δ 整体少了一个常数。
 * 在云台系（世界角）上直接补，与 k 的符号/量程无关。 */
constexpr float kYawDeltaTrim = 112.5f * 3.14159265f / 180.0f;  ///< +112.5°

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

  /* 零位不能在这里记：此时电机多半还没回过帧，angle() 还是 0，
   * 记下来 δ 就等于关节绝对角加了一个随机常量。改为 update() 里第一帧反馈到达时记。 */
  yaw_zero_ = 0.0f;
  yaw_zero_valid_ = false;
  delta_ = 0.0f;

  /* 方向/量程自标定状态清零；k 先用经验值，标定成功后覆盖 */
  yaw_scale_ = -1.0f;
  sign_latched_ = false;
  sum_jj_ = 0.0f;
  sum_jy_ = 0.0f;
  sum_yy_ = 0.0f;
  sum_travel_ = 0.0f;
  last_joint_ = yaw_fb_;
  last_imu_yaw_ = 0.0f;

  /* 摇杆目标起点与当前角一致，接管时不跳 */
  stick_yaw_ = yaw_fb_;
  stick_pitch_ = target_.pitch;
  stick_active_ = false;

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

void Gimbal::updateStick(float lv, float lh, bool enabled)
{
  if (!enabled) {
    stick_active_ = false;
    return;
  }

  if (!stick_active_) {
    /* 接管瞬间从当前反馈角起积分，避免跳变 */
    stick_yaw_ = yaw_fb_;
    stick_active_ = true;
  }

  /* yaw：速度积分，松杆停在当前位置 */
  stick_yaw_ += kYawStickGain * lh * kDt;

  /* pitch：直接角度映射，限位与 setTarget() 一致 */
  stick_pitch_ = hello_world::Bound(-kPitchMax * lv, kPitchMin, kPitchMax);
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

  /* 第一帧关节反馈到达后才记录零位：之前 angle() 还是 0 */
  if (!yaw_zero_valid_) {
    if (static_cast<hello_world::motor::DaMiao *>(yaw_motor)->hasFeedback()) {
      yaw_zero_ = yaw_fb_;
      yaw_zero_valid_ = true;
      /* 目标同步到当前角，避免位置环把关节拉回 0 */
      target_.yaw = yaw_fb_;
      stick_yaw_ = yaw_fb_;
      last_joint_ = yaw_fb_;
      auto *imu = GetImu();
      last_imu_yaw_ = (imu != nullptr) ? imu->yaw() : 0.0f;
    }
  } else {
    updateYawScale(yaw_fb_);
  }

  /* δ 的符号与量程都在 yaw_scale_ 里，不能直接拿关节角差当 δ */
  delta_ = yaw_scale_ * (yaw_fb_ - yaw_zero_) + kYawDeltaTrim;

  /* 摇杆接管时目标来自摇杆，否则用 setTarget() 下发的目标 */
  const float yaw_goal = stick_active_ ? stick_yaw_ : target_.yaw;
  const float pitch_goal = stick_active_ ? stick_pitch_ : target_.pitch;

  /* yaw：把目标投影到当前圈，误差落在 [-π, π) */
  const float yaw_target_near =
      yaw_fb_ + hello_world::AngleNormRad(yaw_goal - yaw_fb_);
  yaw_out_ = yaw_pid->calc(yaw_target_near, yaw_fb_, kDt);

  /* pitch：目标已在 setTarget() / updateStick() 钳位，这里再钳一次 */
  const float pitch_target =
      hello_world::Bound(pitch_goal, kPitchMin, kPitchMax);
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

  /* 交还目标控制权：下次接管先用当时的反馈角对齐 */
  stick_active_ = false;

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

void Gimbal::updateYawScale(float yaw_joint)
{
  auto *imu = GetImu();

  /* IMU 未标定：只跟着刷新基准，不采样 */
  if (imu == nullptr || !imu->isCalibrated()) {
    last_joint_ = yaw_joint;
    last_imu_yaw_ = (imu != nullptr) ? imu->yaw() : 0.0f;
    return;
  }

  const float dj = yaw_joint - last_joint_;
  const float dy = hello_world::AngleNormRad(imu->yaw() - last_imu_yaw_);
  last_joint_ = yaw_joint;
  last_imu_yaw_ = imu->yaw();

  if (sign_latched_) {
    return;
  }

  /* 关节没动时 dy 全是噪声；跳变时 dy 混进了丢帧，都不采样 */
  const float abs_dj = (dj < 0.0f) ? -dj : dj;
  if (abs_dj < kCalMinStep || abs_dj > kCalMaxStep) {
    return;
  }

  sum_jj_ += dj * dj;
  sum_jy_ += dj * dy;
  sum_yy_ += dy * dy;
  sum_travel_ += abs_dj;

  if (sum_travel_ < kCalMinTravel) {
    return;
  }

  /* k = Σ(j·y)/Σ(j·j)，R² 用 Σ 形式算，不必保存每个样本 */
  const float k = sum_jy_ / sum_jj_;
  const float abs_k = (k < 0.0f) ? -k : k;
  float r2 = 0.0f;
  if (sum_yy_ > 0.0f) {
    r2 = (sum_jy_ * sum_jy_) / (sum_jj_ * sum_yy_);
  }

  if (r2 < kCalMinR2 || abs_k < kCalMinScale || abs_k > kCalMaxScale) {
    /* 样本被底盘转动污染或量程离谱：清空重来 */
    sum_jj_ = 0.0f;
    sum_jy_ = 0.0f;
    sum_yy_ = 0.0f;
    sum_travel_ = 0.0f;
    return;
  }

  /* 量程也取拟合值：若电机反馈角被 PMAX 缩放，这里会得到 ≈4 或 ≈0.25 */
  yaw_scale_ = k;
  sign_latched_ = true;
}

}  // namespace robot

/* Private function definitions ----------------------------------------------*/
