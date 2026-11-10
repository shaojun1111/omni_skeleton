/**
 *******************************************************************************
 * @file      :robot.hpp
 * @brief     :云台板顶层状态机（Dead / Resurrection / Working）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      lishaojun       1. 三态迁移 + 就绪判定 + 指令下发
 *******************************************************************************
 * @attention :
 *  1. 本类决定“这一毫秒云台电机该不该出力”，Gimbal 只负责把目标变成力矩。
 *  2. update() 先做状态迁移，再调 Gimbal::update()。
 *  3. 遥控离线、底盘离线、IMU 未标定，任一成立都回 Dead。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_ROBOT_MODULES_ROBOT_HPP_
#define GIMBAL_ROBOT_MODULES_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include "module_state.hpp"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace hello_world
{

class OfflineChecker;

namespace imu
{
class Imu;
}  // namespace imu
}  // namespace hello_world

namespace robot
{

class Gimbal;
class GimbalChassisComm;

class Robot
{
 public:
  /**
   * @brief 初始化：状态回 Dead、计数清零
   */
  void init(void);

  /**
   * @brief 1ms 周期入口：迁移状态 → 生成指令 → 下发
   */
  void update(void);

  void registerGimbal(Gimbal *gimbal) { gimbal_ = gimbal; }
  void registerImu(hello_world::imu::Imu *imu) { imu_ = imu; }
  void registerComm(GimbalChassisComm *comm) { comm_ = comm; }
  void registerRemote(hello_world::OfflineChecker *rc_offline)
  {
    rc_offline_ = rc_offline;
  }

  LifeState lifeState(void) const { return state_; }
  uint32_t readyCount(void) const { return ready_cnt_; }

 private:
  /**
   * @brief 四个条件同时成立：IMU 标定完成 / 遥控在线 / 底盘在线 / 底盘自检通过
   */
  bool isReady(void) const;

  void enterDead(void);
  void enterWorking(void);
  void updateResurrection(void);
  void updateWorking(void);

  Gimbal *gimbal_ = nullptr;
  hello_world::imu::Imu *imu_ = nullptr;
  GimbalChassisComm *comm_ = nullptr;
  hello_world::OfflineChecker *rc_offline_ = nullptr;

  LifeState state_ = LifeState::kDead;
  uint32_t ready_cnt_ = 0;
  bool gimbal_inited_ = false;  ///< 电机反馈到达后再做一次角度对齐
};

}  // namespace robot

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif /* GIMBAL_ROBOT_MODULES_ROBOT_HPP_ */
