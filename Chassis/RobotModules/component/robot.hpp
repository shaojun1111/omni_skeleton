/**
 *******************************************************************************
 * @file      :robot.hpp
 * @brief     :底盘板顶层状态机（Dead / Resurrection / Working）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      lishaojun       1. 三态迁移 + 就绪判定 + 指令下发
 *  V1.1.0      2026-11-11      <author>        1. 就绪条件并入本地遥控；下发底盘模式
 *******************************************************************************
 * @attention :
 *  1. 本类决定“这一毫秒电机该不该出力”，Chassis 只负责把目标变成力矩。
 *  2. update() 先做状态迁移，再调 Chassis::update()。
 *  3. 急停、对方离线、IMU 未标定，任一成立都回 Dead。
 *  4. 底盘模式随 GimbalData::mode 每周期下发，由 Chassis 决定 w 的来源。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_ROBOT_MODULES_ROBOT_HPP_
#define CHASSIS_ROBOT_MODULES_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include "module_state.hpp"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types --------------------------------------------------------*/
namespace hello_world
{
namespace imu
{
class Imu;
}  // namespace imu
}  // namespace hello_world

namespace robot
{

class Chassis;
class GimbalChassisComm;

class Robot
{
 public:
  /**
   * @brief 初始化：状态回 Dead、计数清零、底盘模块初始化
   */
  void init(void);

  /**
   * @brief 1ms 周期入口：迁移状态 → 生成指令 → 下发
   */
  void update(void);

  void registerChassis(Chassis *chassis) { chassis_ = chassis; }
  void registerImu(hello_world::imu::Imu *imu) { imu_ = imu; }
  void registerComm(GimbalChassisComm *comm) { comm_ = comm; }

  LifeState lifeState(void) const { return state_; }
  uint32_t readyCount(void) const { return ready_cnt_; }

 private:
  /**
   * @brief 本地遥控是否在线（DBUS 帧未超时）
   */
  bool isRemoteOnline(void) const;

  /**
   * @brief 四个条件同时成立：IMU 标定完成 / 对方在线 /（本地遥控 或 对方转发的
   *        remote_ok）/ 对方自检通过
   */
  bool isReady(void) const;

  void enterDead(void);
  void enterWorking(void);
  void updateResurrection(void);
  void updateWorking(void);

  Chassis *chassis_ = nullptr;
  hello_world::imu::Imu *imu_ = nullptr;
  GimbalChassisComm *comm_ = nullptr;

  LifeState state_ = LifeState::kDead;
  uint32_t ready_cnt_ = 0;
};

}  // namespace robot

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif /* CHASSIS_ROBOT_MODULES_ROBOT_HPP_ */
