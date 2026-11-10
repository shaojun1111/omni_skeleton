/**
 *******************************************************************************
 * @file      :ins_motor.cpp
 * @brief     :底盘 4 个 M3508 轮电机的实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-21      lishaojun       1. 初版
 *******************************************************************************
 * @attention :
 *  1. ID 按实车接线配置（当前 1~4）。
 *  2. dir（方向）等基地标定后再填。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#include "ins_motor.hpp"

#include "motor.hpp"

namespace {

hello_world::motor::OptionalParams MakeOpt(hello_world::motor::Dir dir) {
  hello_world::motor::OptionalParams opt;
  opt.dir = dir;
  /* 减速比以实车为准：手转标定 14:1（2026-10-22 第一次基地调试、四轮能跑）。
   * 考核书 2.3 表写的是 3591/187≈19.2，与实车不符，按实车填 14，
   * 否则轮速反馈会被折算错、闭环在错误的速度上收敛。 */
  opt.remove_build_in_reducer = true;
  opt.ex_redu_rat = 14.0f;
  return opt;
}

}  // namespace

hello_world::motor::Motor *GetMotorWheel(int idx) {
  static hello_world::motor::M3508 wheels[4] = {
      hello_world::motor::M3508(1, MakeOpt(hello_world::motor::kDirRev)),
      hello_world::motor::M3508(2, MakeOpt(hello_world::motor::kDirRev)),
      hello_world::motor::M3508(3, MakeOpt(hello_world::motor::kDirFwd)),
      hello_world::motor::M3508(4, MakeOpt(hello_world::motor::kDirFwd)),
  };

  if (idx < 0 || idx >= 4) {
    return nullptr;
  }
  return &wheels[idx];
}

/* Private function definitions ----------------------------------------------*/
