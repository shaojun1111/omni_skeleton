/**
 *******************************************************************************
 * @file      :ins_comm.hpp
 * @brief     :通讯对象实例化（遥控器 DT7 + 离线检测）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-11      lishaojun       1. 补齐 DT7 单例与遥控离线检查器
 *******************************************************************************
 * @attention :
 *  1. DT7 只负责解码 18 字节 DBUS，串口收发在 Task/component/comm_task.cpp。
 *  2. 离线检查器由 UART 回调 update()，由 Robot 状态机读取。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_INSTANCE_INS_COMM_HPP_
#define CHASSIS_INSTANCE_INS_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include "DT7.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace hello_world {
class OfflineChecker;
}  // namespace hello_world

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取遥控器解码对象（DT7）单例。 */
hello_world::remote_control::DT7 *GetRemoteControl();

/* 获取遥控离线检查器单例（阈值 100ms）。 */
hello_world::OfflineChecker *GetRemoteOffline();

#endif /* CHASSIS_INSTANCE_INS_COMM_HPP_ */
