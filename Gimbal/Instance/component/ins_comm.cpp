/**
 *******************************************************************************
 * @file      :ins_comm.cpp
 * @brief     :通讯对象实例化（遥控器 DT7 + 离线检测）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-26      lishaojun       1. 初版：DT7 单例
 *  V1.1.0      2026-11-10      lishaojun       2. 离线检查器移到本文件，供 Robot 读取
 *******************************************************************************
 * @attention :
 *  1. DT7 只负责解码 18 字节 DBUS，串口收发在 Task/component/comm_task.cpp。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_comm.hpp"

#include "offline_checker.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 遥控离线检测：100ms 未收到新帧即离线（初始为离线） */
static hello_world::OfflineChecker g_rc_offline(100);

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::remote_control::DT7 *GetRemoteControl() {
  static hello_world::remote_control::DT7 rc;
  return &rc;
}

hello_world::OfflineChecker *GetRemoteOffline() { return &g_rc_offline; }

/* Private function definitions ----------------------------------------------*/
