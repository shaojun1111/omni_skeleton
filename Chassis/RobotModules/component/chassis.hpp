/**
 *******************************************************************************
 * @file      :chassis.hpp
 * @brief     :底盘运动模块（逆解 + 四轮速度环）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-10-29      lishaojun       1. 初版：麦轮逆解 + 轮速闭环
 *******************************************************************************
 * @attention :
 *  1. setTarget() 只保存目标，实际解算在 update() 里做。
 *  2. update() 每 1ms 调用一次。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_ROBOT_MODULES_CHASSIS_HPP_
#define CHASSIS_ROBOT_MODULES_CHASSIS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "module_state.hpp"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace robot
{

class Chassis
{
 public:
  /**
   * @brief 初始化：清空目标、复位 4 个 PID、电机输入置 0
   */
  void init(void);

  /**
   * @brief 下发目标速度
   * @param target vx(m/s) / vy(m/s) / w(rad/s)，云台系
   */
  void setTarget(const ChassisState& target);

  /**
   * @brief 周期任务：逆解 → 限幅 → 速度环 → setInput
   * @note  每 1ms 调用一次
   */
  void update(void);

  /**
   * @brief 立即停车：目标清零、电机输入置 0、PID 复位
   */
  void stop(void);

  /* 调试用：读取最近一次解算的轮缘速度 */
  float wheelSpeed(int idx) const
  {
    return (idx >= 0 && idx < 4) ? wheel_speed_[idx] : 0.0f;
  }

 private:
  ChassisState target_;
  float wheel_speed_[4] = {0.0f, 0.0f, 0.0f, 0.0f};  ///< [RF, RB, LB, LF]，m/s
};

}  // namespace robot

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif /* CHASSIS_ROBOT_MODULES_CHASSIS_HPP_ */
