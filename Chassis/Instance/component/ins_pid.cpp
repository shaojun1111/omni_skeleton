/**
 *******************************************************************************
 * @file      :ins_pid.cpp
 * @brief     :PID 实例（四路轮速环）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-10-29      lishaojun       1. 实现 GetPidMotorWheel()
 *  V1.1.0      2026-11-11      <author>        1. 新增跟随环 GetPidChassisFollow()
 *******************************************************************************
 * @attention :
 *  1. 输出单位为 M3508 电流码值（±16384 对应 ±20A）。
 *  2. 参数为初值，基地按实际响应调整。
 *  3. 跟随环输出为车体自旋角速度 w（rad/s，逆时针为正）。
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

/* 跟随环参数：kp 单位 (rad/s)/rad，ki 单位 (rad/s)/(rad·s)。
 * 输出已在 pid 内对称限幅，与 chassis.cpp 的 kMaxFollowW 取同一个量级；
 * 车头追云台太肉就加 kFollowKp，来回摆就减 kFollowKp。
 * kd 默认为 0：微分项按 (measure 差 / dt) 计算，1 ms 周期下放大约 1000 倍，
 * 真要加阻尼请从 0.001~0.01 量级试，不要照搬位置环的 0.5。 */
constexpr float kFollowKp = 8.0f;
constexpr float kFollowKi = 1.0f;
constexpr float kFollowKd = 0.0f;
constexpr float kFollowOutLim = 6.0f;
constexpr float kFollowILim = 3.0f;

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

hello_world::pid::Pid *GetPidChassisFollow() {
  static hello_world::pid::Pid pid(kFollowKp, kFollowKi, kFollowKd,
                                  kFollowOutLim, kFollowILim);
  return &pid;
}

/* Private function definitions ----------------------------------------------*/
