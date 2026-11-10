/**
 *******************************************************************************
 * @file      :robot.cpp
 * @brief     :底盘板顶层状态机实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      lishaojun       1. 三态迁�?+ 就绪判定 + 指令下发
 *******************************************************************************
 * @attention :
 *  1. 迁移条件集中�?isReady()，状态函数只负责动作�? *  2. 所有状态都�?chassis_->update()，保�?CAN 帧连续�? *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "robot.hpp"

#include "chassis.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace
{

constexpr uint32_t kReadyHoldCnt = 100;  ///< 就绪条件需连续保持的周期数�?ms/周期�?
}  // namespace

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
namespace robot
{

void Robot::init(void)
{
  state_ = LifeState::kDead;
  ready_cnt_ = 0;

  if (chassis_ != nullptr) {
    chassis_->init();
    chassis_->stop();
  }
}

void Robot::update(void)
{
  switch (state_) {
    case LifeState::kDead: {
      /* 急停未松开时停�?Dead；松开后自动进 Resurrection 重新自检 */
      const bool estop = (comm_ != nullptr) && comm_->gimbal_data().estop;
      if (!estop) {
        state_ = LifeState::kResurrection;
        ready_cnt_ = 0;
      }
      break;
    }
    case LifeState::kResurrection:
      updateResurrection();
      break;
    case LifeState::kWorking:
      updateWorking();
      break;
    default:
      enterDead();
      break;
  }

  /* 所有状态都下发一次，保证 CAN 帧和 PID 连续 */
  if (chassis_ != nullptr) {
    chassis_->update();
  }
}

/* Private function definitions ----------------------------------------------*/
bool Robot::isReady(void) const
{
  if (imu_ == nullptr || comm_ == nullptr) {
    return false;
  }
  if (!imu_->isCalibrated()) {
    return false;
  }
  if (comm_->isOffline()) {
    return false;
  }
  const auto &g = comm_->gimbal_data();
  return g.remote_ok && g.gimbal_ok;
}

void Robot::enterDead(void)
{
  state_ = LifeState::kDead;
  ready_cnt_ = 0;
  if (chassis_ != nullptr) {
    chassis_->stop();
  }
}

void Robot::enterWorking(void)
{
  state_ = LifeState::kWorking;
  if (chassis_ != nullptr) {
    /* �?0 起步，避免沿用上一状态残留的目标 */
    chassis_->setTarget(ChassisState{});
  }
}

void Robot::updateResurrection(void)
{
  if (chassis_ != nullptr) {
    chassis_->stop();
  }

  if (!isReady()) {
    ready_cnt_ = 0;
    return;
  }

  ready_cnt_ = ready_cnt_ + 1;
  if (ready_cnt_ >= kReadyHoldCnt) {
    enterWorking();
  }
}

void Robot::updateWorking(void)
{
  if (chassis_ == nullptr || comm_ == nullptr) {
    enterDead();
    return;
  }

  const auto &g = comm_->gimbal_data();
  if (g.estop || !isReady()) {
    enterDead();
    return;
  }

  chassis_->setDelta(g.delta);
  chassis_->setTarget(ChassisState{g.vx, g.vy, g.spin});
}

}  // namespace robot
