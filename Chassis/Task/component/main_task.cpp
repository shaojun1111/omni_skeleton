/**
 *******************************************************************************
 * @file      :main_task.cpp
 * @brief     :主任务（1ms 周期入口 + TIM6 中断分发）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-21      lishaojun       1. 初版：最小调度 + CAN 通讯启动
 *******************************************************************************
 * @attention :
 *  1. MainTaskInit() 由 main() 在 while(1) 之前调用一次。
 *  2. MainTask/CommTask 由 TIM6 1ms 中断调用（先算后发）。
 *  3. MOTOR_CALIB_TEST 是临时标定开关，标定完必须改回 0！
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"
#include "comm_task.hpp"

#include "tim.h"

#include "ins_motor.hpp"

/* Private macro -------------------------------------------------------------*/
/* 临时电机标定开关：
 *   0 = 关闭（正常运行，电机输入保持 0）
 *   1 = 打开（上电后 4 个轮子依次正转 1s / 反转 1s）
 * ⚠️ 标定完必须改回 0，否则上电自转，违反安全红线！ */
#define MOTOR_CALIB_TEST 0

/* Exported function definitions ---------------------------------------------*/
void MainTaskInit(void) {
  /* 阶段 1：先把通讯跑起来（第 6/7 课再补 IMU、遥控、Robot） */
  CommTaskInit();
}

void MainTask(void) {
#if MOTOR_CALIB_TEST
  static int test_cnt = 0;

  /* 每拍先把 4 个电机输入清零，避免上一个电机的值残留 */
  for (int i = 0; i < 4; ++i) {
    auto *m = GetMotorWheel(i);
    if (m != nullptr) {
      m->setInput(0.0f);
    }
  }

  if (test_cnt < 8000) {                    // 总共 8 秒
    const int idx = test_cnt / 2000;        // 0~3，每个电机 2 秒
    const int phase = (test_cnt % 2000) / 1000;  // 0=正转 1 秒，1=反转 1 秒

    auto *m = GetMotorWheel(idx);
    if (m != nullptr) {
      m->setInput(phase == 0 ? 1500.0f : -1500.0f);
    }
    ++test_cnt;
  }
#else
  /* TODO: 第 12 课接 GetRobot()->update() */
#endif
}

/* Private function definitions ----------------------------------------------*/
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) {
    MainTask();   // 先"算"
    CommTask();   // 后"发" + 喂狗
  }
}
