/**
 *******************************************************************************
 * @file      :ins_chassis_gimbal_comm.cpp
 * @brief     :双板通讯对象实例化（云台板）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-09      lishaojun       1. 云台板单例：tx 0x211 / rx 0x210
 *******************************************************************************
 * @attention :
 *  1. 云台板发 G2C（0x211），收 C2G（0x210）。
 *  2. ID 与底盘板 ins_chassis_gimbal_comm.cpp 必须互换。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_chassis_gimbal_comm.hpp"
#include "gimbal_chassis_comm.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* 本板发送 / 接收帧 ID */
constexpr uint32_t kC2gId = 0x210;  ///< 底盘 → 云台
constexpr uint32_t kG2cId = 0x211;  ///< 云台 → 底盘
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm *GetGimbalChassisComm() {
  static robot::GimbalChassisComm comm(
      robot::GimbalChassisComm::CodePart::Gimbal, kG2cId, kC2gId);
  return &comm;
}

/* Private function definitions ----------------------------------------------*/
