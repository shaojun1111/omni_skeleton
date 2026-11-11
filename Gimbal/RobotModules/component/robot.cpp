/**
 *******************************************************************************
 * @file      :robot.cpp
 * @brief     :云台板顶层状态机实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      lishaojun       1. 三态迁移 + 就绪判定 + 指令下发
 *******************************************************************************
 * @attention :
 *  1. 迁移条件集中在 isReady()，状态函数只负责动作。
 *  2. 所有状态都走 gimbal_->update()，保证 CAN 帧连续。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "robot.hpp"

#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "offline_checker.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace
{

constexpr uint32_t kReadyHoldCnt = 100;  ///< 就绪条件需连续保持的周期数（1ms/周期）

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
  gimbal_inited_ = false;
}

void Robot::update(void)
{
  switch (state_) {
    case LifeState::kDead: {
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

  if (gimbal_ != nullptr) {
    gimbal_->update();
  }
}

/* Private function definitions ----------------------------------------------*/
bool Robot::isReady(void) const
{
  if (imu_ == nullptr || comm_ == nullptr || rc_offline_ == nullptr) {
    return false;
  }
  if (!imu_->isCalibrated()) {
    return false;
  }
  /* 本板 DBUS 或底盘转发（接收机在底盘板）任一在线即可 */
  if (rc_offline_->isOffline() && !comm_->chassis_data().remote_ok) {
    return false;
  }
  if (comm_->isOffline()) {
    return false;
  }
  return comm_->chassis_data().chassis_ok;
}

void Robot::enterDead(void)
{
  state_ = LifeState::kDead;
  ready_cnt_ = 0;
  if (gimbal_ != nullptr) {
    gimbal_->stop();
  }
}

void Robot::enterWorking(void)
{
  state_ = LifeState::kWorking;
  if (gimbal_ != nullptr) {
    /* 目标对齐当前角，避免切状态瞬间跳变 */
    gimbal_->setTarget(
        GimbalState{gimbal_->yawFeedback(), gimbal_->pitchFeedback()});
  }
}

void Robot::updateResurrection(void)
{
  if (gimbal_ != nullptr) {
    /* 第一帧反馈到达后再对齐角度；之前 angle() 还是初值 0 */
    if (!gimbal_inited_) {
      gimbal_->init();
      gimbal_inited_ = true;
    }
    gimbal_->stop();
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
  if (gimbal_ == nullptr || comm_ == nullptr) {
    enterDead();
    return;
  }

  if (comm_->gimbal_data().estop || !isReady()) {
    enterDead();
    return;
  }

  /* 第 14 课：遥控 → 目标角。现在锁定当前角。 */
  gimbal_->setTarget(
      GimbalState{gimbal_->yawFeedback(), gimbal_->pitchFeedback()});
}

}  // namespace robot
