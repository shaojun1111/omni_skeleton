/**
 *******************************************************************************
 * @file      : HW_can.cpp
 * @brief     : CAN 收发基础封装实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-18      Hello World     1. 考核骨架：CanFilter_Init + CAN_Send_Msg
 *                                                全量提供，接收分发回调掏空
 *  V1.1.0      2026-10-27      lishaojun       1. 补全云台 DM4310 反馈分发（0x11/0x12）
 *                                                与接收调试计数
 *******************************************************************************
 * @attention :
 *  1. 达妙反馈 ID = 电机 id + 0x10：pitch(id=1) → 0x11，yaw(id=2) → 0x12。
 *  2. 两路 CAN 的 FIFO0/FIFO1 回调都做分发：底盘板实测 CAN2 的帧会落到 FIFO0，
 *     云台板不做这个假设，四个分支全接。
 *  3. 回调里必须调用 HAL_CAN_GetRxMessage 把帧取走，否则 FMP0 恒非 0，
 *     中断会反复触发（HAL 不会自动清）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"

#include "can.h"

#include "ins_motor.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* 达妙反馈帧 ID（motor_DaMiao.hpp:338 GetRxId = 0x10 + id） */
constexpr uint32_t kPitchRxId = 0x10U + kPitchMotorId;  // 0x11
constexpr uint32_t kYawRxId = 0x10U + kYawMotorId;      // 0x12

/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1;
static CAN_RxHeaderTypeDef rx_header2;
static uint8_t can1_rx_data[8];
static uint8_t can2_rx_data[8];
static uint32_t p_tx_mailbox;

/* 调试计数（Ozone Watched Data 观察，Refresh 需手动改 100ms） */
volatile uint32_t g_can1_rx_cnt = 0;  ///< CAN1 收到的帧数
volatile uint32_t g_can2_rx_cnt = 0;  ///< CAN2 收到的帧数
volatile uint32_t g_dm_rx_cnt = 0;    ///< 命中的达妙反馈帧数

/* Private function prototypes -----------------------------------------------*/
static void DispatchRxFrame(const CAN_RxHeaderTypeDef *header, uint8_t data[8]);

/* Exported function definitions ---------------------------------------------*/

void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef can_filter;

  /* 全收：掩码全 0，任何 ID 均通过 */
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterActivation = ENABLE;

  if (hcan == &hcan1) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterBank = 0;
  } else if (hcan == &hcan2) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    can_filter.FilterBank = 14;
  } else {
    return;
  }

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }
}

void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  tx_header.StdId = id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = len;
  HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &p_tx_mailbox);
}

/**
 * @brief   CAN1 FIFO0 接收中断回调
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    同时兼容 CAN2 的帧落到 FIFO0（底盘板实测行为）
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1,
                             can1_rx_data) == HAL_OK) {
      g_can1_rx_cnt = g_can1_rx_cnt + 1;
      DispatchRxFrame(&rx_header1, can1_rx_data);
    }
  } else if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header2,
                             can2_rx_data) == HAL_OK) {
      g_can2_rx_cnt = g_can2_rx_cnt + 1;
      DispatchRxFrame(&rx_header2, can2_rx_data);
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief   CAN2 FIFO1 接收中断回调
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    同时兼容 CAN1 的帧落到 FIFO1
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header1,
                             can1_rx_data) == HAL_OK) {
      g_can1_rx_cnt = g_can1_rx_cnt + 1;
      DispatchRxFrame(&rx_header1, can1_rx_data);
    }
  } else if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2,
                             can2_rx_data) == HAL_OK) {
      g_can2_rx_cnt = g_can2_rx_cnt + 1;
      DispatchRxFrame(&rx_header2, can2_rx_data);
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/* Private function definitions ----------------------------------------------*/
/**
 * @brief   按标准帧 ID 把 8 字节分发给对应的上层对象
 * @param   header: 本帧的接收头（提供 StdId）
 * @param   data:   本帧的 8 字节数据
 * @retval  None
 * @note    未命中的 ID 直接丢弃；第 11 课在这里加 0x210（C2G）分支
 */
static void DispatchRxFrame(const CAN_RxHeaderTypeDef *header, uint8_t data[8])
{
  const uint32_t id = header->StdId;

  if (id == kPitchRxId) {
    auto *pitch = GetMotorPitch();
    if (pitch != nullptr) {
      pitch->decode(data);
      g_dm_rx_cnt = g_dm_rx_cnt + 1;
    }
  } else if (id == kYawRxId) {
    auto *yaw = GetMotorYaw();
    if (yaw != nullptr) {
      yaw->decode(data);
      g_dm_rx_cnt = g_dm_rx_cnt + 1;
    }
  }
  /* TODO【第 11 课】: 0x210 = C2G 双板帧 → GetGimbalChassisComm()->decode(data) */
}
