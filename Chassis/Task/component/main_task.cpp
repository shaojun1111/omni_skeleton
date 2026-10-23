/*
 * @Author: shaojun1111 2424624237@qq.com
 * @Date: 2026-09-19 15:36:50
 * @LastEditors: shaojun1111 2424624237@qq.com
 * @LastEditTime: 2026-10-22 18:47:03
 * @FilePath: \omni_skeleton\Chassis\Task\component\main_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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

/* 调试计数器（Ozone 观察，确认调度链路） */
volatile uint32_t g_tim6_cnt = 0;  /* TIM6 中断进了多少次 */
volatile uint32_t g_comm_cnt = 0;  /* CommTask 调了多少次 */

/* Exported function definitions ---------------------------------------------*/
void MainTaskInit(void) {
  /* 阶段 1：先把通讯跑起来（第 6/7 课再补 IMU、遥控、Robot） */
  CommTaskInit();
}

void MainTask(void) {
#if MOTOR_CALIB_TEST
  static int test_cnt = 0;

  if (test_cnt < 3000) {  // 4 轮同时正转 3 秒
    for (int i = 0; i < 4; ++i) {
      auto *m = GetMotorWheel(i);
      if (m != nullptr) {
        m->setInput(600.0f);
      }
    }
    ++test_cnt;
  } else {  // 3 秒后全部停
    for (int i = 0; i < 4; ++i) {
      auto *m = GetMotorWheel(i);
      if (m != nullptr) {
        m->setInput(0.0f);
      }
    }
  }
#else
  /* TODO: 第 12 课接 GetRobot()->update() */
#endif
}

/* Private function definitions ----------------------------------------------*/
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) {
    ++g_tim6_cnt;
    MainTask();   // 先"算"
    ++g_comm_cnt;
    CommTask();   // 后"发" + 喂狗
  }
}
