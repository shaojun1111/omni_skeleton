/**
 *******************************************************************************
 * @file      :chassis_ik_solver.hpp
 * @brief     :麦克纳姆轮逆运动学（车体速度 → 四轮轮缘速度）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-29      lishaojun       1. 初版：45° 滚子麦轮逆解
 *******************************************************************************
 * @attention :
 *  1. 纯数学，无硬件依赖，可直接在 PC 上单测。
 *  2. 输出数组按电机 ID 顺序：[RF, RB, LB, LF]。
 *  3. 几何参数用实车测量值替换。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_INC_CHASSIS_IK_SOLVER_HPP_
#define CHASSIS_INC_CHASSIS_IK_SOLVER_HPP_

/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
namespace hello_world {
namespace chassis_ik_solver {

/* 底盘几何参数（待实机测量后修改） */
constexpr float kR = 0.076f;  ///< 轮半径，m
constexpr float kA = 0.150f;  ///< 半轴距（前后轮中心距的一半），m
constexpr float kB = 0.150f;  ///< 半轮距（左右轮中心距的一半），m
constexpr float kK = kA + kB; ///< 自转等效力臂，m

/* Exported types ------------------------------------------------------------*/

class ChassisIkSolver
{
 public:
  /**
   * @brief 逆解：车体速度 → 四轮轮缘速度
   * @param vx 前进速度，m/s
   * @param vy 左移速度，m/s
   * @param w  逆时针角速度，rad/s
   * @param v_wheel 输出，按 [RF, RB, LB, LF] 顺序，m/s
   */
  void solve(float vx, float vy, float w, float v_wheel[4]) const;
};

}  // namespace chassis_ik_solver
}  // namespace hello_world

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif /* CHASSIS_INC_CHASSIS_IK_SOLVER_HPP_ */
