/**
 *******************************************************************************
 * @file      :ins_robot.cpp
 * @brief     :顶层状态机与底盘模块实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-10      lishaojun       1. 初版：Robot / Chassis 单例
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

#include "chassis.hpp"
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

robot::Chassis *GetChassis() {
  static robot::Chassis chassis;
  return &chassis;
}

/* Private function definitions ----------------------------------------------*/
