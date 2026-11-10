/**
 *******************************************************************************
 * @file      :chassis.cpp
 * @brief     :底盘运动模块实现
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-10-29      lishaojun       1. 逆解 → 限幅 → 速度环 → setInput
 *  V1.1.0      2026-11-11      <author>        1. 三模式：分离 / 跟随 / 小陀螺
 *******************************************************************************
 * @attention :
 *  1. 速度环反馈用 M3508 输出轴角速度（rad/s），与目标值同单位。
 *  2. 限幅在 PID 之前：目标始终物理可达，积分项不会因不可达目标饱和。
 *  3. 符号约定：δ 逆时针为正（云台在车头左侧），w 逆时针为正（车体左转）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "chassis.hpp"

#include "base.hpp"
#include "chassis_ik_solver.hpp"
#include "ins_chassis_iksolver.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
#include "motor.hpp"
#include "pid.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace
{

constexpr float kDt = 0.001f;           ///< 控制周期，s
constexpr float kMaxWheelSpeed = 3.0f;  ///< 轮缘速度上限，m/s
/* δ 符号：实测车体方向与预期相反时改成 -1.0f */
constexpr float kDeltaSign = 1.0f;

constexpr float kMaxVx = 3.0f;  ///< 车体前后速度上限，m/s
constexpr float kMaxVy = 3.0f;  ///< 车体左右速度上限，m/s
/* 自旋上限取 8.0，比云台侧 spin 编码上限（±6.35 rad/s）宽，不截断正常指令 */
constexpr float kMaxW = 8.0f;
/* 跟随模式回正角速度上限，rad/s；车头追尾摆得太大或太肉时现场调这个 */
constexpr float kMaxFollowW = 6.0f;

}  // namespace

/* 跟随环 PID 实例在 ins_pid.cpp 中定义，此处局部声明 */
hello_world::pid::Pid *GetPidChassisFollow(void);

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
namespace robot
{

void Chassis::init(void)
{
  target_ = ChassisState{};
  body_cmd_ = ChassisState{};
  delta_ = 0.0f;
  mode_ = ChassisMode::kSeparate;
  enabled_ = false;

  auto *follow_pid = GetPidChassisFollow();
  if (follow_pid != nullptr) {
    follow_pid->reset();
  }

  for (int i = 0; i < 4; ++i) {
    wheel_speed_[i] = 0.0f;

    auto *pid = GetPidMotorWheel(i);
    if (pid != nullptr) {
      pid->reset();
    }

    auto *motor = GetMotorWheel(i);
    if (motor != nullptr) {
      motor->setInput(0.0f);
    }
  }
}

void Chassis::setTarget(const ChassisState& target)
{
  target_ = target;
  enabled_ = true;
}

void Chassis::setMode(ChassisMode mode)
{
  if (mode == mode_) {
    return;
  }

  mode_ = mode;

  /* 换模式时清掉跟随环的积分和微分历史，切进跟随时不会先猛打一下 */
  auto *follow_pid = GetPidChassisFollow();
  if (follow_pid != nullptr) {
    follow_pid->reset();
  }
}

void Chassis::setDelta(float delta)
{
  delta_ = hello_world::AngleNormRad(delta);
}

void Chassis::update(void)
{
  auto *solver = GetChassisIkSolver();
  if (solver == nullptr) {
    return;
  }

  /* 1. 云台系 → 车体系 */
  float vx_b = 0.0f;
  float vy_b = 0.0f;
  float w = 0.0f;

  if (enabled_) {
    hello_world::chassis_ik_solver::rotateToBody(
        target_.vx, target_.vy, kDeltaSign * delta_, &vx_b, &vy_b);

    /* 2. 按模式解出 w：平移都是云台系旋转到车体系，差别只在自旋项 */
    switch (mode_) {
      case ChassisMode::kFollow: {
        auto *follow_pid = GetPidChassisFollow();
        if (follow_pid != nullptr) {
          /* 跟随：车头追云台朝向，目标 δ = 0。
           * δ > 0 说明云台偏到车头左侧，车头要逆时针(+)转才追得上；
           * PID 按 e = target − measure 求误差，所以把 −δ 当作测量值送入，
           * 输出即 +w。被控对象 dδ/dt = −w，闭环收敛到 δ = 0。 */
          w = follow_pid->calc(0.0f, -delta_, kDt);
          w = hello_world::Bound(w, -kMaxFollowW, kMaxFollowW);
        }
        break;
      }
      case ChassisMode::kSeparate:
      case ChassisMode::kSpin:
      default:
        /* 分离 / 小陀螺：w 直接取云台下发的 spin，不做自动回正；
         * 两者的差别在云台侧摇杆怎么映射到 spin，底盘侧都是直通。 */
        w = target_.w;
        break;
    }

    /* 3. 进逆解前限幅 */
    vx_b = hello_world::Bound(vx_b, -kMaxVx, kMaxVx);
    vy_b = hello_world::Bound(vy_b, -kMaxVy, kMaxVy);
    w = hello_world::Bound(w, -kMaxW, kMaxW);
  } else {
    /* 未使能（Dead / estop / 离线）：走原来的归零路径，模式解算不参与 */
    auto *follow_pid = GetPidChassisFollow();
    if (follow_pid != nullptr) {
      follow_pid->reset();
    }
  }

  body_cmd_ = ChassisState{vx_b, vy_b, w};

  /* 4. 逆解：车体速度 → 四轮轮缘速度 */
  solver->solve(vx_b, vy_b, w, wheel_speed_);

  for (int i = 0; i < 4; ++i) {
    auto *motor = GetMotorWheel(i);
    auto *pid = GetPidMotorWheel(i);
    if (motor == nullptr || pid == nullptr) {
      continue;
    }

    /* 5. 限幅 */
    const float v = hello_world::Bound(wheel_speed_[i], -kMaxWheelSpeed,
                                       kMaxWheelSpeed);

    /* 6. 轮缘速度 → 轮子角速度 */
    const float omega_target = v / hello_world::chassis_ik_solver::kR;

    /* 7. 速度环：目标角速度 vs 反馈角速度 → 电流码值 */
    const float current = pid->calc(omega_target, motor->vel(), kDt);

    /* 8. 写回电机对象，由 CommTask 的 encode() 发出 */
    motor->setInput(current);
  }
}

void Chassis::stop(void)
{
  target_ = ChassisState{};
  body_cmd_ = ChassisState{};
  enabled_ = false;

  auto *follow_pid = GetPidChassisFollow();
  if (follow_pid != nullptr) {
    follow_pid->reset();
  }

  for (int i = 0; i < 4; ++i) {
    wheel_speed_[i] = 0.0f;

    auto *motor = GetMotorWheel(i);
    if (motor != nullptr) {
      motor->setInput(0.0f);
    }

    auto *pid = GetPidMotorWheel(i);
    if (pid != nullptr) {
      pid->reset();
    }
  }
}

}  // namespace robot

/* Private function definitions ----------------------------------------------*/
