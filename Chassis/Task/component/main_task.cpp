/**
 *******************************************************************************
 * @file      :main_task.cpp
 * @brief     :主任务（1ms 周期入口 + TIM6 中断分发）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-21      lishaojun       1. 初版：最小调度 + CAN 通讯启动
 *  V1.1.0      2026-10-28      lishaojun       2. 接入 IMU：上电初始化 + 每 1ms 更新
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

#include "ins_imu.hpp"
#include "ins_motor.hpp"

#include "imu.hpp"

/* Private macro -------------------------------------------------------------*/
/* 临时电机标定开关：
 *   0 = 关闭（正常运行，电机输入保持 0）
 *   1 = 打开（上电后 4 个轮子同时正转 3 秒）
 * ⚠️ 标定完必须改回 0，否则上电自转，违反安全红线！ */
#define MOTOR_CALIB_TEST 0

/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 调试计数器（Ozone 观察，确认调度链路） */
volatile uint32_t g_tim6_cnt = 0;  /* TIM6 中断进了多少次 */
volatile uint32_t g_comm_cnt = 0;  /* CommTask 调了多少次 */

/* IMU 调试变量（Ozone Watched Data 观察，Refresh 需手动改 100ms） */
volatile float g_dbg_roll = 0.0f;          ///< 横滚角，rad
volatile float g_dbg_pitch = 0.0f;         ///< 俯仰角，rad
volatile float g_dbg_yaw = 0.0f;           ///< 偏航角，rad
volatile uint8_t g_dbg_imu_calibrated = 0; ///< 0=标定中 1=完成
volatile uint8_t g_dbg_imu_err = 0;        ///< BMI088 初始化错误位

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
void MainTaskInit(void) {
  /* IMU 先初始化（含阻塞式 SPI 配置），再启动通讯 */
  auto *imu = GetImu();
  if (imu != nullptr) {
    imu->init();
  }

  CommTaskInit();
}

void MainTask(void) {
  /* ---- IMU 更新 ---- */
  auto *imu = GetImu();
  if (imu != nullptr) {
    imu->update();
    g_dbg_roll = imu->roll();
    g_dbg_pitch = imu->pitch();
    g_dbg_yaw = imu->yaw();
    g_dbg_imu_calibrated = imu->isCalibrated() ? 1 : 0;
    g_dbg_imu_err = imu->errState();
  }

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
    g_tim6_cnt = g_tim6_cnt + 1;
    MainTask();   // 先"算"
    g_comm_cnt = g_comm_cnt + 1;
    CommTask();   // 后"发" + 喂狗
  }
}
