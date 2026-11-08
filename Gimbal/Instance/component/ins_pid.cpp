/**
 *******************************************************************************
 * @file      :ins_pid.cpp
 * @brief     :PID 实例（云台 yaw / pitch 位置环）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-08      lishaojun       1. 实现 GetPidMotorYaw/GetPidMotorPitch
 *******************************************************************************
 * @attention :
 *  1. 输入角度差 rad，输出力矩 N·m；量程受 DM-J4310 的 TMAX=7.5 限制。
 *  2. 参数为初值，基地按实际响应调整（先 Kp，再加 Kd，最后按需加 Ki）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_pid.hpp"

#include "pid.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace {

/* yaw：惯量小、行程无限，刚度可以给大一些 */
constexpr float kYawKp = 8.0f;
constexpr float kYawKi = 0.0f;
constexpr float kYawKd = 0.35f;
constexpr float kYawOutLim = 3.0f;
constexpr float kYawILim = 1.0f;

/* pitch：带重力负载，Kp 略小、Kd 略小；静差明显时再加 Ki */
constexpr float kPitchKp = 6.0f;
constexpr float kPitchKi = 0.0f;
constexpr float kPitchKd = 0.30f;
constexpr float kPitchOutLim = 2.5f;
constexpr float kPitchILim = 1.0f;

}  // namespace

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::pid::Pid *GetPidMotorYaw() {
  static hello_world::pid::Pid pid(kYawKp, kYawKi, kYawKd, kYawOutLim,
                                   kYawILim);
  return &pid;
}

hello_world::pid::Pid *GetPidMotorPitch() {
  static hello_world::pid::Pid pid(kPitchKp, kPitchKi, kPitchKd,
                                   kPitchOutLim, kPitchILim);
  return &pid;
}

/* Private function definitions ----------------------------------------------*/
