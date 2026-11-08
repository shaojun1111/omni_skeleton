/**
 *******************************************************************************
 * @file      :gimbal.hpp
 * @brief     :云台运动模块（yaw / pitch 两轴位置环）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-08      lishaojun       1. 初版：角度差 → PID → MIT 力矩
 *******************************************************************************
 * @attention :
 *  1. yaw 连续旋转，目标角在 update() 里投影到当前圈再交给 PID。
 *  2. pitch 在 setTarget() 里做软件限位，防止顶结构。
 *  3. update() 每 1ms 调用一次。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_ROBOT_MODULES_GIMBAL_HPP_
#define GIMBAL_ROBOT_MODULES_GIMBAL_HPP_

/* Includes ------------------------------------------------------------------*/
#include "module_state.hpp"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace robot
{

class Gimbal
{
 public:
  /**
   * @brief 初始化：目标角设为当前反馈角、复位 PID、输出 0 力矩
   */
  void init(void);

  /**
   * @brief 下发目标关节角
   * @param target yaw / pitch，单位 rad
   * @note  pitch 在内部钳到 [kPitchMin, kPitchMax]
   */
  void setTarget(const GimbalState& target);

  /**
   * @brief 周期任务：读反馈 → 算误差 → PID → setMitInput
   * @note  每 1ms 调用一次
   */
  void update(void);

  /**
   * @brief 立即停止：输出 0 力矩、PID 复位、目标回到当前角
   */
  void stop(void);

  /* 调试用：最近一次反馈与输出 */
  float yawFeedback(void) const { return yaw_fb_; }
  float pitchFeedback(void) const { return pitch_fb_; }
  float yawOutput(void) const { return yaw_out_; }
  float pitchOutput(void) const { return pitch_out_; }

 private:
  GimbalState target_{};
  float yaw_fb_ = 0.0f;     ///< yaw 反馈角，rad
  float pitch_fb_ = 0.0f;   ///< pitch 反馈角，rad
  float yaw_out_ = 0.0f;    ///< yaw 输出力矩，N·m
  float pitch_out_ = 0.0f;  ///< pitch 输出力矩，N·m
};

}  // namespace robot

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif /* GIMBAL_ROBOT_MODULES_GIMBAL_HPP_ */
