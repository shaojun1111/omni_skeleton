/**
 *******************************************************************************
 * @file      :chassis.cpp
 * @brief     :底盘运动模块实现
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-10-29      lishaojun       1. 逆解 → 限幅 → 速度环 → setInput
 *******************************************************************************
 * @attention :
 *  1. 速度环反馈用 M3508 输出轴角速度（rad/s），与目标值同单位。
 *  2. 限幅在 PID 之前：目标始终物理可达，积分项不会因不可达目标饱和。
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

}  // namespace

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
}

void Chassis::update(void)
{
  auto *solver = GetChassisIkSolver();
  if (solver == nullptr) {
    return;
  }

  /* 1. 逆解：车体速度 → 四轮轮缘速度 */
  solver->solve(target_.vx, target_.vy, target_.w, wheel_speed_);

  for (int i = 0; i < 4; ++i) {
    auto *motor = GetMotorWheel(i);
    auto *pid = GetPidMotorWheel(i);
    if (motor == nullptr || pid == nullptr) {
      continue;
    }

    /* 2. 限幅 */
    const float v = hello_world::Bound(wheel_speed_[i], -kMaxWheelSpeed,
                                       kMaxWheelSpeed);

    /* 3. 轮缘速度 → 轮子角速度 */
    const float omega_target = v / hello_world::chassis_ik_solver::kR;

    /* 4. 速度环：目标角速度 vs 反馈角速度 → 电流码值 */
    const float current = pid->calc(omega_target, motor->vel(), kDt);

    /* 5. 写回电机对象，由 CommTask 的 encode() 发出 */
    motor->setInput(current);
  }
}

void Chassis::stop(void)
{
  target_ = ChassisState{};

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
