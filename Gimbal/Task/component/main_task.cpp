/**
 *******************************************************************************
 * @file      :main_task.cpp
 * @brief     :主任务（1ms 周期入口 + TIM6 中断分发）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-26      lishaojun       1. 初版：最小调度 + 通讯启动
 *******************************************************************************
 * @attention :
 *  1. MainTaskInit() 由 main() 在 while(1) 之前调用一次。
 *  2. MainTask/CommTask 由 TIM6 1ms 中断调用（先算后发）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"
#include "comm_task.hpp"

#include "tim.h"

/* Exported function definitions ---------------------------------------------*/
void MainTaskInit(void) {
  /* 阶段 1：先把通讯跑起来（后续再补 IMU、Robot） */
  CommTaskInit();
}

void MainTask(void) {
  /* TODO: 第 12 课接 GetRobot()->update() */
}

/* Private function definitions ----------------------------------------------*/
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) {
    MainTask();   // 先"算"
    CommTask();   // 后"发" + 喂狗
  }
}
