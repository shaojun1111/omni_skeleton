/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     :通讯任务（CAN 收发 + 遥控 DBUS 接收 + 喂狗）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-21      lishaojun       1. 初版：CAN 启动 + 电机发送
 *  V1.1.0      2026-11-11      lishaojun       2. 补齐 USART3 空闲中断 + DMA 接收 DT7
 *******************************************************************************
 * @attention :
 *  1. CAN1/CAN2 分工是"工作假设"，基地确认后可能要改。
 *  2. 4 个 M3508 共用同一个 8 字节 buffer，只发一次 0x200。
 *  3. 遥控接收机接 USART3（100kbps / 9bit / 偶校验）；回调末尾必须重启接收，
 *     否则只收到一帧。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "comm_task.hpp"

#include "can.h"
#include "iwdg.h"
#include "usart.h"

#include "DT7.hpp"
#include "HW_can.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_comm.hpp"
#include "ins_imu.hpp"
#include "ins_motor.hpp"
#include "ins_robot.hpp"
#include "motor.hpp"
#include "offline_checker.hpp"
#include "robot.hpp"

/* 调试用：Ozone 里直接观察 4 个轮子的角度/速度（基地标定用） */
volatile float g_dbg_angle[4] = {0.0f};
volatile float g_dbg_vel[4] = {0.0f};

/* 调试计数器：CAN_Send_Msg 调了多少次 */
volatile uint32_t g_send_cnt = 0;
volatile uint32_t g_comm_tx_cnt = 0;   ///< 双板帧发送次数
volatile uint32_t g_comm_tx_fail = 0;  ///< 双板帧发送失败次数

/* 调试寄存器：CAN1 发送/错误状态 */
volatile uint32_t g_dbg_can1_tsr = 0;   ///< CAN1->TSR，发送状态
volatile uint32_t g_dbg_can1_esr = 0;   ///< CAN1->ESR，错误状态
volatile uint32_t g_dbg_can1_free = 0;  ///< CAN1 空闲发送邮箱数（0~3）

/* ---- 遥控 DBUS ---- */
/* 接收缓冲（18 字节），DMA 直接写入，无需手动搬运 */
static uint8_t g_dbus_buf[hello_world::remote_control::kRcRxDataLen];
volatile uint32_t g_dbus_rx_cnt = 0;    ///< 解码成功帧数（正常约 500/s）
volatile uint8_t g_dbg_rc_offline = 1;  ///< 遥控离线标志：1=离线 0=在线

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

  /* ④ 遥控 DBUS：USART3 + DMA + 空闲中断（IWDG 已由 main() 的 MX_IWDG_Init 启动） */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_buf,
                               hello_world::remote_control::kRcRxDataLen);
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

  /* ---- 双板帧：C2G（0x210），底盘 → 云台 ---- */
  auto *comm = GetGimbalChassisComm();
  if (comm != nullptr) {
    auto *imu = GetImu();
    if (imu != nullptr) {
      comm->chassis_data().chassis_yaw = imu->yaw();
      comm->chassis_data().imu_ok = imu->isCalibrated();
      /* 自检通过 = IMU 零漂完成。不能写成“在 Working”，否则两板互相等待。 */
      comm->chassis_data().chassis_ok = imu->isCalibrated();
    }
    auto *robot = GetRobot();
    comm->chassis_data().life_state =
        (robot != nullptr) ? robot->lifeState() : robot::LifeState::kDead;

    uint8_t comm_buf[8] = {0};
    comm->encode(comm_buf);
    CAN_Send_Msg(&hcan2, comm_buf, comm->tx_id(), 8);
    g_comm_tx_cnt = g_comm_tx_cnt + 1;
  }

  /* ---- 调试：CAN1 发送状态与错误状态 ---- */
  g_dbg_can1_tsr = CAN1->TSR;
  g_dbg_can1_esr = CAN1->ESR;
  g_dbg_can1_free = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);

  /* ---- 遥控离线状态（SWD 观察；UART 回调负责 update()） ---- */
  auto *rc_offline = GetRemoteOffline();
  g_dbg_rc_offline = (rc_offline != nullptr && rc_offline->isOffline()) ? 1 : 0;

  /* ---- 喂狗 ---- */
  HAL_IWDG_Refresh(&hiwdg);
}

/* Private function definitions ----------------------------------------------*/
/**
 * @brief UART 空闲/半满回调：收满 18 字节后解码 DBUS
 * @note  该回调是 HAL 弱函数，必须 extern "C"；末尾必须重启接收，否则只收一帧。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t Size) {
  if (huart != &huart3) {
    return;
  }

  if (Size == hello_world::remote_control::kRcRxDataLen) {
    auto *rc = GetRemoteControl();
    if (rc != nullptr) {
      rc->decode(g_dbus_buf);
      auto *rc_offline = GetRemoteOffline();
      if (rc_offline != nullptr) {
        rc_offline->update();
      }
      g_dbus_rx_cnt = g_dbus_rx_cnt + 1;
    }
  }

  /* 无论本帧是否完整，都重启下一轮接收 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_buf,
                               hello_world::remote_control::kRcRxDataLen);
}
