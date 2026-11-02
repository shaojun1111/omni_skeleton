/**
 *******************************************************************************
 * @file      :chassis_ik_solver.hpp
 * @brief     :麦克纳姆轮逆运动学（车体速度 → 四轮轮缘速度）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-29      lishaojun       1. 初版：45° 滚子麦轮逆解
 *  V1.1.0      2026-11-02      lishaojun       2. 加云端系 → 车体系旋转（可 PC 单测）
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
#include <cmath>

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

/**
 * @brief 把云台系的平移速度旋转到车体系
 * @param vx_g 云台系前进速度，m/s
 * @param vy_g 云台系左移速度，m/s
 * @param delta 云台相对车头的偏航角，逆时针为正，rad
 * @param vx_b 输出：车体系 x 速度
 * @param vy_b 输出：车体系 y 速度
 * @note  绕 z 轴旋转，w 不参与
 */
inline void rotateToBody(float vx_g, float vy_g, float delta, float* vx_b,
                         float* vy_b)
{
  const float c = std::cos(delta);
  const float s = std::sin(delta);
  *vx_b = vx_g * c - vy_g * s;
  *vy_b = vx_g * s + vy_g * c;
}

}  // namespace chassis_ik_solver
}  // namespace hello_world

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif /* CHASSIS_INC_CHASSIS_IK_SOLVER_HPP_ */
