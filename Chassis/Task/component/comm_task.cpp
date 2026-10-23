/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     :通讯任务（CAN 收发 + 喂狗）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-21      lishaojun       1. 初版：CAN 启动 + 电机发送
 *******************************************************************************
 * @attention :
 *  1. CAN1/CAN2 分工是"工作假设"，基地确认后可能要改。
 *  2. 4 个 M3508 共用同一个 8 字节 buffer，只发一次 0x200。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "comm_task.hpp"

#include "can.h"
#include "iwdg.h"

#include "HW_can.hpp"
#include "ins_motor.hpp"
#include "motor.hpp"

/* 调试用：Ozone 里直接观察 4 个轮子的角度/速度（基地标定用） */
volatile float g_dbg_angle[4] = {0.0f};
volatile float g_dbg_vel[4] = {0.0f};

/* 调试计数器：CAN_Send_Msg 调了多少次 */
volatile uint32_t g_send_cnt = 0;

/* Exported function definitions ---------------------------------------------*/
void CommTaskInit(void) {
  /* ① 配置滤波器（全收） */
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);

  /* ② 启动 CAN 外设（漏了 = 完全不能收发） */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);

  /* ③ 使能接收中断（四路全开，避免帧落到未使能的 FIFO） */
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  /* TODO: 遥控器 UART 接收、IWDG 启动（第 6 课） */
}

void CommTask(void) {
  /* ---- 底盘：4 个 M3508 共用 0x200 ---- */
  uint8_t motor_buf[8] = {0};
  bool has_motor = false;

  for (int i = 0; i < 4; ++i) {
    auto *m = GetMotorWheel(i);
    if (m != nullptr) {
      m->encode(motor_buf);  // 每个电机只填自己那 2 字节
      g_dbg_angle[i] = m->angle();
      g_dbg_vel[i] = m->vel();
      has_motor = true;
    }
  }

  if (has_motor) {
    CAN_Send_Msg(&hcan2, motor_buf, 0x200, 8);  // ⚠️ CAN2 为工作假设
    ++g_send_cnt;
  }

  /* ---- 双板帧（第 11 课再加） ---- */
  // auto *comm = GetGimbalChassisComm();
  // if (comm != nullptr) {
  //   uint8_t comm_buf[8] = {0};
  //   comm->encode(comm_buf);
  //   CAN_Send_Msg(&hcan1, comm_buf, 0x210, 8);
  // }

  /* ---- 喂狗 ---- */
  HAL_IWDG_Refresh(&hiwdg);
}

/* Private function definitions ----------------------------------------------*/
/* TODO【新人任务】:
 * 实现 UART 空闲中断回调 HAL_UARTEx_RxEventCallback：
 *   - 收到满 18 字节后调用 DT7.decode() 解包遥控器数据，并重新启动下一轮接收；
 *   - 该回调同样是 C 链接（extern "C"）。
 * 注意：CAN 的接收分发在 src/HW_can.cpp 的两个回调里（电机 decode / 双板协议解码），
 *      不要在这里重复写 CAN 接收逻辑。
 */
