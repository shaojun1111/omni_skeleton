/**
 *******************************************************************************
 * @file      :ins_pid.cpp
 * @brief     :PID 实例（四路轮速环）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-10-29      lishaojun       1. 实现 GetPidMotorWheel()
 *******************************************************************************
 * @attention :
 *  1. 输出单位为 M3508 电流码值（±16384 对应 ±20A）。
 *  2. 参数为初值，基地按实际响应调整。
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

/* 速度环参数：kp 单位 码值/(rad/s)，ki 单位 码值/(rad/s·s) */
constexpr float kKp = 1500.0f;
constexpr float kKi = 2000.0f;
constexpr float kKd = 0.0f;
constexpr float kOutLim = 8000.0f;
constexpr float kILim = 8000.0f;

}  // namespace

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::pid::Pid *GetPidMotorWheel(int idx) {
  static hello_world::pid::Pid pids[4] = {
      hello_world::pid::Pid(kKp, kKi, kKd, kOutLim, kILim),
      hello_world::pid::Pid(kKp, kKi, kKd, kOutLim, kILim),
      hello_world::pid::Pid(kKp, kKi, kKd, kOutLim, kILim),
      hello_world::pid::Pid(kKp, kKi, kKd, kOutLim, kILim),
  };

  if (idx < 0 || idx >= 4) {
    return nullptr;
  }
  return &pids[idx];
}

/* Private function definitions ----------------------------------------------*/
