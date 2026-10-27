/**
 *******************************************************************************
 * @file      :ins_motor.hpp
 * @brief     :云台关节电机实例（DM-J4310）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-10-27      lishaojun       1. 补全 yaw/pitch 电机实例与 ID/默认 MIT 常量
 *******************************************************************************
 * @attention :
 *  1. 达妙反馈 ID = 电机 id + 0x10；发送 ID = 电机 id（MIT 与命令帧同 ID）。
 *  2. 达妙上电默认失能，必须由 comm_task 每周期调用 encode() 发使能命令。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_INSTANCE_INS_MOTOR_HPP_
#define GIMBAL_INSTANCE_INS_MOTOR_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "motor.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* 云台关节电机 ID（组长确认；反馈 ID = id + 0x10 → pitch 0x11 / yaw 0x12） */
constexpr uint8_t kPitchMotorId = 1;
constexpr uint8_t kYawMotorId = 2;

/* 上电默认 MIT 刚度/阻尼：先给很小值，云台会缓慢转向电机零点。
 * 基地② 标定完方向后，再按实测逐级加大（kp: 5 → 20 → 50 → 100）。 */
constexpr float kGimbalDefaultKp = 5.0f;
constexpr float kGimbalDefaultKd = 0.2f;

/* Exported types --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取云台 yaw 关节电机单例（DM-J4310）。 */
hello_world::motor::Motor *GetMotorYaw();
/* 获取云台 pitch 关节电机单例（DM-J4310）。 */
hello_world::motor::Motor *GetMotorPitch();

#endif /* GIMBAL_INSTANCE_INS_MOTOR_HPP_ */
