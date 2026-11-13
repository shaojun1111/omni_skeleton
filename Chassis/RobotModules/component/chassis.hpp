/**
 *******************************************************************************
 * @file      :chassis.hpp
 * @brief     :底盘运动模块（逆解 + 四轮速度环）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-10-29      lishaojun       1. 初版：麦轮逆解 + 轮速闭环
 *  V1.1.0      2026-11-11      <author>        1. 三模式：分离 / 跟随 / 小陀螺
 *******************************************************************************
 * @attention :
 *  1. setTarget() 只保存目标，实际解算在 update() 里做。
 *  2. update() 每 1ms 调用一次。
 *  3. 模式决定 w 的来源：分离 / 小陀螺用云台下发的 spin，跟随用 δ 闭环回正。
 *  4. setTarget() 之后才允许出力，stop() 后 update() 输出全零，防止停车状态下
 *     跟随环继续驱动车体。
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
   * @brief 设置云台相对车头的偏航角
   * @param delta 单位 rad，逆时针为正；内部归一化到 [-π, π)
   */
  void setDelta(float delta);

  /**
   * @brief 设置底盘模式
   * @param mode kSeparate 分离 / kFollow 跟随 / kSpin 小陀螺
   * @note  切换模式时自动复位跟随环，避免残留积分冲一下车体
   */
  void setMode(ChassisMode mode);

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

  /* 调试用：读取当前 δ */
  float delta(void) const { return delta_; }

  /* 调试用：读取最近一次下发的云台系目标（未旋转、未限幅） */
  ChassisState target(void) const { return target_; }

  /* 调试用：读取当前模式 */
  ChassisMode mode(void) const { return mode_; }

  /**
   * @brief 调试用：读取最近一次进逆解的车体系命令（已限幅）
   * @note  vx 前 / vy 左 / w 逆时针为正，可直接和轮速对照验算
   */
  ChassisState bodyCmd(void) const { return body_cmd_; }

 private:
  ChassisState target_;  ///< 云台系目标（vx / vy / spin），来自 GimbalData
  ChassisState body_cmd_;  ///< 车体系命令，模式解算 + 限幅之后
  float wheel_speed_[4] = {0.0f, 0.0f, 0.0f, 0.0f};  ///< [RF, RB, LB, LF]，m/s
  float delta_ = 0.0f;  ///< 云台相对车头的偏航角，rad
  float w_cmd_ = 0.0f;  ///< 分离/小陀螺的自旋指令斜坡输出，rad/s
  ChassisMode mode_ = ChassisMode::kSeparate;  ///< 当前底盘模式
  bool enabled_ = false;  ///< setTarget 置位 / stop 清零，为假时输出全零
};

}  // namespace robot

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif /* CHASSIS_ROBOT_MODULES_CHASSIS_HPP_ */
