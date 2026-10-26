/**
 *******************************************************************************
 * @file      :ins_comm.cpp
 * @brief     :通讯对象实例化（遥控器 DT7）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-26      lishaojun       1. 初版：DT7 单例
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

#include "DT7.hpp"

/* Exported function definitions ---------------------------------------------*/
hello_world::remote_control::DT7 *GetRemoteControl() {
  static hello_world::remote_control::DT7 rc;
  return &rc;
}

/* Private function definitions ----------------------------------------------*/
