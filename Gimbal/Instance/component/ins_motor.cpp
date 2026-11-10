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
 *     若改成 kNegPiToPosPi，decode() 里的圈数会被 fmod 清掉（motor_DaMiao.cpp:221）。
 *  3. DM-J4310 上电失能，不发指令不反馈（考核书 2.3）：若 CAN1 上始终只有 yaw 的 0x12
 *     而没有 pitch 的 0x11，依次检查发送 ID 是否为 0x01、ESC_ID 是否为 1、
 *     CTRL_MODE 是否为 MIT，最后才怀疑接线与供电。
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
  opt.input_type = hello_world::motor::InputType::kTorq;  // 达妙 MIT 只接受力矩输入
  opt.remove_build_in_reducer = false;  // 保留内置减速器：反馈即关节输出轴
  opt.ex_redu_rat = 1.0f;               // 无外置减速
  opt.angle_offset = 0.0f;              // 零点待基地用 setAngleValue() 标定
  opt.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf;
  return opt;
}

/**
 * @brief 单个电机的上电引导状态
 */
struct BootState {
  bool enable_sent = false;  ///* 使能帧是否已排队
  bool mit_inited = false;   ///* 初始 MIT 目标是否已写入
};

/**
 * @brief 上电引导：先发使能帧，收到「已使能」反馈后再写一帧初始 MIT 目标
 * @param motor 达妙电机指针
 * @param state 该电机的上电引导状态
 * @note  DM-J4310 上电默认失能，不发指令既不动作也不反馈（考核书 2.3），
 *        因此上电后第一个周期就必须把使能帧（0xFC，发送 ID 为电机 id）排进队列；
 *        MIT 目标等反馈表明已使能后再写，位置取当前反馈角，避免使能瞬间被拉向 0 rad。
 *        kp/kd 取与 yaw 同量级（kGimbalDefaultKp/kGimbalDefaultKd），kp=0 时电机不受
 *        重力约束，等同没使能。
 */
void BootJointMotor(hello_world::motor::DaMiao *motor, BootState *state) {
  if (!state->enable_sent) {
    motor->enable();
    state->enable_sent = true;
  }
  if (!state->mit_inited && motor->is_enabled()) {
    motor->setMitInput(
        motor->angle(), 0.0f, kGimbalDefaultKp, kGimbalDefaultKd, 0.0f);
    state->mit_inited = true;
  }
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::motor::Motor *GetMotorYaw() {
  /* 函数内 static：首次调用时构造，之后全程序唯一 */
  static hello_world::motor::DM_J4310 yaw(
      kYawMotorId, MakeGimbalOpt(hello_world::motor::kDirFwd),
      /* auto_enable = */ true);

  /* 初始目标只写一次，避免覆盖后续控制环设置的目标 */
  static BootState boot;
  BootJointMotor(&yaw, &boot);
  return &yaw;
}

hello_world::motor::Motor *GetMotorPitch() {
  static hello_world::motor::DM_J4310 pitch(
      kPitchMotorId, MakeGimbalOpt(hello_world::motor::kDirFwd),
      /* auto_enable = */ true);

  static BootState boot;
  BootJointMotor(&pitch, &boot);
  return &pitch;
}

/* Private function definitions ----------------------------------------------*/
