/**
 *******************************************************************************
 * @file      :main_task.cpp
 * @brief     :主任务（1ms 周期入口 + TIM6 中断分发）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-26      lishaojun       1. 初版：最小调度 + 通讯启动
 *  V1.1.0      2026-10-28      lishaojun       2. 接入 IMU：上电初始化 + 每 1ms 更新
 *  V1.2.0      2026-11-08      lishaojun       3. 接入 Gimbal：两轴位置环
 *  V1.3.0      2026-11-10      lishaojun       4. 接入 Robot 状态机
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

#include "ins_chassis_gimbal_comm.hpp"
#include "ins_comm.hpp"
#include "ins_imu.hpp"
#include "ins_robot.hpp"

#include "gimbal.hpp"
#include "imu.hpp"
#include "robot.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* IMU 调试变量（Ozone Watched Data 观察，Refresh 需手动设 100ms） */
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

/* 状态机调试变量 */
volatile uint8_t g_dbg_life_state = 0;   ///< 0=Dead 1=Resurrection 2=Working
volatile uint32_t g_dbg_ready_cnt = 0;   ///< 就绪条件连续满足的周期数

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

  /* 注册模块并初始化状态机 */
  auto *robot = GetRobot();
  if (robot != nullptr) {
    robot->registerGimbal(GetGimbal());
    robot->registerImu(GetImu());
    robot->registerComm(GetGimbalChassisComm());
    robot->registerRemote(GetRemoteOffline());
    robot->init();
  }
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

  /* ---- 状态机 ---- */
  auto *robot = GetRobot();
  if (robot != nullptr) {
    robot->update();
    g_dbg_life_state = static_cast<uint8_t>(robot->lifeState());
    g_dbg_ready_cnt = robot->readyCount();
  }

  /* ---- 云台调试变量 ---- */
  auto *gimbal = GetGimbal();
  if (gimbal != nullptr) {
    g_dbg_gimbal_yaw_fb = gimbal->yawFeedback();
    g_dbg_gimbal_pitch_fb = gimbal->pitchFeedback();
    g_dbg_gimbal_yaw_out = gimbal->yawOutput();
    g_dbg_gimbal_pitch_out = gimbal->pitchOutput();
  }
}

/* Private function definitions ----------------------------------------------*/
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) {
    MainTask();   // 先“算”
    CommTask();   // 后“发” + 喂狗
  }
}
