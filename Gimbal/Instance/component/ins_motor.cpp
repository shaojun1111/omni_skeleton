/**
 *******************************************************************************
 * @file      :ins_motor.cpp
 * @brief     :云台关节电机实例（DM-J4310）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-10-27      lishaojun       1. 实现 GetMotorYaw/GetMotorPitch（MIT 初始目标）
 *******************************************************************************
 * @attention :
 *  1. DM-J4310 自带减速器且输出轴即关节轴 → 不移除内置减速器、不叠加外部减速比。
 *  2. angle_range 必须用 kNegInfToPosInf：库不做归一化，angle() 才能累计多圈；
 *     若改成 kNegPiToPosPi，decode() 里的圈数会被 fmod 清掉（motor_DaMiao.cpp:201）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_motor.hpp"

#include "motor.hpp"
#include "motor_DaMiao.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
namespace {

/**
 * @brief 生成云台关节电机的可选参数
 * @param dir 电机安装方向
 * @retval 组装好的可选参数
 * @note  方向先统一给 kDirFwd；基地② 用「小幅位置指令」实测后再翻转。
 */
hello_world::motor::OptionalParams MakeGimbalOpt(hello_world::motor::Dir dir) {
  hello_world::motor::OptionalParams opt;
  opt.dir = dir;
  opt.remove_build_in_reducer = false;  // 保留内置减速器：反馈即关节输出轴
  opt.ex_redu_rat = 1.0f;               // 无外置减速
  opt.angle_offset = 0.0f;              // 零点待基地用 setAngleValue() 标定
  opt.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf;
  return opt;
}

/**
 * @brief 写入一次初始 MIT 目标（位置 0 / 速度 0 / 小刚度 / 无前馈）
 * @param motor 达妙电机指针
 * @note  不写的话 comm_data_.kp 默认 0，电机使能后不出力，基地无法观察运动。
 */
void ApplyDefaultMit(hello_world::motor::DaMiao *motor) {
  motor->setMitInput(0.0f, 0.0f, kGimbalDefaultKp, kGimbalDefaultKd, 0.0f);
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::motor::Motor *GetMotorYaw() {
  /* 函数内 static：首次调用时构造，之后全程序唯一 */
  static hello_world::motor::DM_J4310 yaw(
      kYawMotorId, MakeGimbalOpt(hello_world::motor::kDirFwd),
      /* auto_enable = */ true);

  /* 初始目标只写一次，避免覆盖后续控制环设置的目标 */
  static bool inited = false;
  if (!inited) {
    ApplyDefaultMit(&yaw);
    inited = true;
  }
  return &yaw;
}

hello_world::motor::Motor *GetMotorPitch() {
  static hello_world::motor::DM_J4310 pitch(
      kPitchMotorId, MakeGimbalOpt(hello_world::motor::kDirFwd),
      /* auto_enable = */ true);

  static bool inited = false;
  if (!inited) {
    ApplyDefaultMit(&pitch);
    inited = true;
  }
  return &pitch;
}

/* Private function definitions ----------------------------------------------*/
