/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     :通讯任务（CAN 初始化 + DBUS 接收 + 喂狗）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-26      lishaojun       1. 初版：USART3 + DMA 接收 DT7
 *******************************************************************************
 * @attention :
 *  1. 遥控接收机在云台板，接 USART3（100kbps / 9bit / 偶校验）。
 *  2. 回调末尾必须重新启动 DMA 接收，否则只收一帧。
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
#include "offline_checker.hpp"

/* Private macro -------------------------------------------------------------*/
/* 云台电机所在 CAN 总线。
 * ⚠️ 基地② 用 g_can1_rx_cnt / g_can2_rx_cnt 确认后改这里：
 *    哪路计数随电机反馈增长，就把这里改成那一路。
 *    默认 hcan1（与底盘板对称：底盘 CAN2=电机，云台 CAN1=电机）。 */
#define GIMBAL_MOTOR_CAN (&hcan1)

/* 板间通讯所在 CAN 总线：与电机总线相反。
 * 底盘板实测 CAN2=电机、CAN1=板间；云台板按对称假设 CAN1=电机、CAN2=板间。 */
#define GIMBAL_COMM_CAN (&hcan2)

/* Private variables ---------------------------------------------------------*/
/* DBUS 接收缓冲（18 字节） */
static uint8_t g_dbus_buf[hello_world::remote_control::kRcRxDataLen];

/* 遥控离线检测：100ms 未收到新帧即离线（初始为离线） */
static hello_world::OfflineChecker g_rc_offline(100);

/* 调试变量（Ozone 观察） */
volatile float g_dbg_rc_lv = 0.0f;
volatile float g_dbg_rc_lh = 0.0f;
volatile float g_dbg_rc_rv = 0.0f;
volatile float g_dbg_rc_rh = 0.0f;
volatile float g_dbg_rc_wheel = 0.0f;
volatile uint8_t g_dbg_rc_l_sw = 0;
volatile uint8_t g_dbg_rc_r_sw = 0;
volatile uint8_t g_dbg_rc_offline = 1;
volatile uint32_t g_dbus_rx_cnt = 0;
volatile uint32_t g_gimbal_tx_cnt = 0;  ///< 云台电机帧发送次数（2 帧/周期）
volatile uint32_t g_gimbal_tx_fail = 0; ///< encode() 失败次数（非 0 说明模式未就绪）
volatile uint32_t g_comm_tx_cnt = 0;    ///< 双板帧发送次数
volatile float g_dbg_delta = 0.0f;      ///< 发给底盘的 delta（调试）

/* Private function prototypes -----------------------------------------------*/
static void SendGimbalMotors(void);

/* Exported function definitions ---------------------------------------------*/
void CommTaskInit(void) {
  /* ---- CAN ---- */
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  /* ---- DBUS：USART3 + DMA + 空闲中断 ---- */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_buf,
                               hello_world::remote_control::kRcRxDataLen);
}

void CommTask(void) {
  /* 刷新调试变量 */
  auto *rc = GetRemoteControl();
  if (rc != nullptr) {
    g_dbg_rc_lv = rc->rc_lv();
    g_dbg_rc_lh = rc->rc_lh();
    g_dbg_rc_rv = rc->rc_rv();
    g_dbg_rc_rh = rc->rc_rh();
    g_dbg_rc_wheel = rc->rc_wheel();
    g_dbg_rc_l_sw = static_cast<uint8_t>(rc->rc_l_switch());
    g_dbg_rc_r_sw = static_cast<uint8_t>(rc->rc_r_switch());
  }
  g_dbg_rc_offline = g_rc_offline.isOffline() ? 1 : 0;

  /* ---- 云台电机：每周期 encode + 发送 ---- */
  /* 达妙上电默认失能，encode() 内部会自动发 0xFC 使能命令，
   * 收到 kMotorEnabled 反馈后才改为发 MIT 位置帧。 */
  SendGimbalMotors();

  /* ---- 双板帧：G2C（0x211），云台 → 底盘 ---- */
  auto *comm = GetGimbalChassisComm();
  if (comm != nullptr) {
    auto *imu = GetImu();
    const float gimbal_yaw = (imu != nullptr) ? imu->yaw() : 0.0f;

    /* delta = 云台朝向 − 车体朝向。
     * 对方离线时归零：此时 chassis_yaw 还是上次的值（或初值 0），
     * 拿来当基准会让底盘朝错误方向平移。 */
    comm->gimbal_data().delta =
        comm->isOffline()
            ? 0.0f
            : (gimbal_yaw - comm->chassis_data().chassis_yaw);
    comm->gimbal_data().remote_ok = !g_rc_offline.isOffline();
    /* 第 12 课：gimbal_ok / estop / mode 改由状态机写入 */
    comm->gimbal_data().gimbal_ok = true;
    /* vx / vy / spin：第 14 课由遥控映射填入，现在保持 0 */

    uint8_t comm_buf[8] = {0};
    comm->encode(comm_buf);
    CAN_Send_Msg(GIMBAL_COMM_CAN, comm_buf, comm->tx_id(), 8);
    g_comm_tx_cnt = g_comm_tx_cnt + 1;
    g_dbg_delta = comm->gimbal_data().delta;
  }

  /* ---- 喂狗 ---- */
  HAL_IWDG_Refresh(&hiwdg);
}

/* Private function definitions ----------------------------------------------*/
/**
 * @brief 编码并发送两路云台电机指令
 * @retval None
 * @note  1. 达妙发送 ID = 电机 id（MIT 帧与命令帧同 ID，见 motor_DaMiao.cpp:350）。
 *        2. encode() 返回 false 表示控制模式未就绪（上电瞬间），此时不发帧。
 *        3. 每周期必须调用：使能命令也是靠它发出去的。
 */
static void SendGimbalMotors(void) {
  uint8_t buf[8] = {0};

  /* pitch: id = 1 */
  auto *pitch = GetMotorPitch();
  if (pitch != nullptr) {
    if (pitch->encode(buf)) {
      CAN_Send_Msg(GIMBAL_MOTOR_CAN, buf, static_cast<uint32_t>(kPitchMotorId),
                   8);
      g_gimbal_tx_cnt = g_gimbal_tx_cnt + 1;
    } else {
      g_gimbal_tx_fail = g_gimbal_tx_fail + 1;
    }
  }

  /* yaw: id = 2 */
  auto *yaw = GetMotorYaw();
  if (yaw != nullptr) {
    if (yaw->encode(buf)) {
      CAN_Send_Msg(GIMBAL_MOTOR_CAN, buf, static_cast<uint32_t>(kYawMotorId), 8);
      g_gimbal_tx_cnt = g_gimbal_tx_cnt + 1;
    } else {
      g_gimbal_tx_fail = g_gimbal_tx_fail + 1;
    }
  }
}

/**
 * @brief UART 空闲/半满回调：收满 18 字节后解码 DBUS
 * @note  该回调是 HAL 弱函数，必须 extern "C"
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t Size) {
  if (huart == &huart3) {
    if (Size == hello_world::remote_control::kRcRxDataLen) {
      auto *rc = GetRemoteControl();
      if (rc != nullptr) {
        rc->decode(g_dbus_buf);
        g_rc_offline.update();
        g_dbus_rx_cnt = g_dbus_rx_cnt + 1;
      }
    }
    /* 无论是否成功，都重启下一轮接收 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_buf,
                                 hello_world::remote_control::kRcRxDataLen);
  }
}
