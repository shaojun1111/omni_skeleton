/**
 *******************************************************************************
 * @file      :main_task.cpp
 * @brief     :主任务（1ms 周期入口 + TIM6 中断分发）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-26      lishaojun       1. 初版：最小调度 + 通讯启动
 *  V1.1.0      2026-10-28      lishaojun       2. 接入 IMU：上电初始化 + 每 1ms 更新
 *  V1.2.0      2026-11-08      lishaojun       3. 接入 Gimbal：两轴位置环
 *******************************************************************************
 * @attention :
 *  1. MainTaskInit() 由 main() 在 while(1) 之前调用一次。
 *  2. MainTask/CommTask 由 TIM6 1ms 中断调用（先算后发）。
 *  3. 本课云台目标固定为上电位置，遥控映射在第 14 课接入。
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
#include "ins_robot.hpp"

#include "gimbal.hpp"
#include "imu.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* IMU 调试变量（Ozone Watched Data 观察，Refresh 需手动改 100ms） */
volatile float g_dbg_roll = 0.0f;          ///< 横滚角，rad
volatile float g_dbg_pitch = 0.0f;         ///< 俯仰角，rad
volatile float g_dbg_yaw = 0.0f;           ///< 偏航角，rad
volatile uint8_t g_dbg_imu_calibrated = 0; ///< 0=标定中 1=完成
volatile uint8_t g_dbg_imu_err = 0;        ///< BMI088 初始化错误位

/* 云台调试变量 */
volatile float g_dbg_gimbal_yaw_fb = 0.0f;     ///< yaw 反馈角，rad
volatile float g_dbg_gimbal_pitch_fb = 0.0f;   ///< pitch 反馈角，rad
volatile float g_dbg_gimbal_yaw_out = 0.0f;    ///< yaw 输出力矩，N·m
volatile float g_dbg_gimbal_pitch_out = 0.0f;  ///< pitch 输出力矩，N·m

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
  auto *imu = GetImu();
  if (imu != nullptr) {
    imu->update();
    g_dbg_roll = imu->roll();
    g_dbg_pitch = imu->pitch();
    g_dbg_yaw = imu->yaw();
    g_dbg_imu_calibrated = imu->isCalibrated() ? 1 : 0;
    g_dbg_imu_err = imu->errState();
  }

  auto *gimbal = GetGimbal();
  if (gimbal != nullptr) {
    /* 第一次进入时对齐当前角，之后目标由本课固定值 / 第 14 课遥控决定 */
    static bool gimbal_inited = false;
    if (!gimbal_inited) {
      gimbal->init();
      gimbal_inited = true;
    }

    gimbal->update();

    g_dbg_gimbal_yaw_fb = gimbal->yawFeedback();
    g_dbg_gimbal_pitch_fb = gimbal->pitchFeedback();
    g_dbg_gimbal_yaw_out = gimbal->yawOutput();
    g_dbg_gimbal_pitch_out = gimbal->pitchOutput();
  }
}

/* Private function definitions ----------------------------------------------*/
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) {
    MainTask();   // 先"算"
    CommTask();   // 后"发" + 喂狗
  }
}
