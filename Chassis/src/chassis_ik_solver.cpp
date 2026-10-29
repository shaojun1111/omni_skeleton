/**
 *******************************************************************************
 * @file      :chassis_ik_solver.cpp
 * @brief     :麦克纳姆轮逆运动学实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-29      lishaojun       1. 初版
 *******************************************************************************
 * @attention :
 *  轮位坐标（x 前，y 左）：
 *    RF ( +a, -b )  ID 1
 *    RB ( -a, -b )  ID 2
 *    LB ( -a, +b )  ID 3
 *    LF ( +a, +b )  ID 4
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "chassis_ik_solver.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
namespace hello_world
{
namespace chassis_ik_solver
{

void ChassisIkSolver::solve(float vx, float vy, float w,
                            float v_wheel[4]) const
{
  v_wheel[0] = vx + vy + kK * w;  // ID 1  RF
  v_wheel[1] = vx - vy + kK * w;  // ID 2  RB
  v_wheel[2] = vx + vy - kK * w;  // ID 3  LB
  v_wheel[3] = vx - vy - kK * w;  // ID 4  LF
}

}  // namespace chassis_ik_solver
}  // namespace hello_world

/* Private function definitions ----------------------------------------------*/
