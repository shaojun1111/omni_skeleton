/**
 *******************************************************************************
 * @file      :ins_robot.cpp
 * @brief     :顶层状态机与云台模块实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      lishaojun       1. 初版：Gimbal 单例
 *  V1.1.0      2026-11-10      lishaojun       2. 新增 Robot 单例
 *******************************************************************************
 * @attention :
 *  1. 本文件只负责造对象、给地址，不写业务逻辑。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_robot.hpp"

#include "gimbal.hpp"
#include "robot.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::Robot *GetRobot() {
  static robot::Robot robot;
  return &robot;
}

robot::Gimbal *GetGimbal() {
  static robot::Gimbal gimbal;
  return &gimbal;
}

/* Private function definitions ----------------------------------------------*/
