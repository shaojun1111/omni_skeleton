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
#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_comm.hpp"
#include "ins_imu.hpp"
#include "ins_motor.hpp"
#include "ins_robot.hpp"
#include "offline_checker.hpp"
#include "robot.hpp"

/* Private macro -------------------------------------------------------------*/
/* 云台电机所在 CAN 总线。
 * ⚠️ 基地② 用 g_can1_rx_cnt / g_can2_rx_cnt 确认后改这里：
 *    哪路计数随电机反馈增长，就把这里改成那一路。
 *    默认 hcan1（与底盘板对称：底盘 CAN2=电机，云台 CAN1=电机）。 */
#define GIMBAL_MOTOR_CAN (&hcan1)

/* 板间通讯所在 CAN 总线。
 * 底盘板 CAN1 能收到云台的 DM 反馈帧 → 两板 CAN1 物理连通 → 板间也走 CAN1。
 * 电机和板间共用 CAN1：ID 不冲突（DM 用 0x01/0x02，双板帧用 0x210/0x211）。 */
#define GIMBAL_COMM_CAN (&hcan2)

/* Private constants ---------------------------------------------------------*/
namespace
{

/* 遥控映射常量 */
constexpr float kMaxLinearSpeed = 2.0f;  ///< 平移速度上限，m/s
/* 自旋上限受协议限制：gimbal_chassis_comm 用 kSpinScale=20 压成 int8，
 * 满量程 127/20≈6.35 rad/s，故上限取 6.0 以免被裁剪。可现场调低。 */
constexpr float kMaxSpin = 6.0f;  ///< 自旋角速度上限，rad/s
constexpr float kStickDeadZone = 0.05f;  ///< 摇杆死区

/* 死区处理：死区内归零，死区外线性重标到满量程，阈值处不跳变 */
float ScaleStick(float v) {
  if (v > kStickDeadZone) {
    return (v - kStickDeadZone) / (1.0f - kStickDeadZone);
  }
  if (v < -kStickDeadZone) {
    return (v + kStickDeadZone) / (1.0f - kStickDeadZone);
  }
  return 0.0f;
}

/* 左拨杆 S1 → 底盘模式。按 DT7 实际档位值（上=1、中=3、下=2）映射：
 * 上档 = 分离（上电默认最安全，摇杆直映射）、中档 = 跟随、下档 = 小陀螺。 */
robot::ChassisMode ModeFromS1(uint8_t sw) {
  switch (sw) {
    case static_cast<uint8_t>(hello_world::remote_control::kSwitchStateUp):
      return robot::ChassisMode::kSeparate;
    case static_cast<uint8_t>(hello_world::remote_control::kSwitchStateMid):
      return robot::ChassisMode::kFollow;
    case static_cast<uint8_t>(hello_world::remote_control::kSwitchStateDown):
      return robot::ChassisMode::kSpin;
    default:
      return robot::ChassisMode::kSeparate;
  }
}

/* 右拨杆 S2 打到下档 = 急停 */
bool EstopFromS2(uint8_t sw) {
  return sw ==
         static_cast<uint8_t>(hello_world::remote_control::kSwitchStateDown);
}

}  // namespace

/* Private variables ---------------------------------------------------------*/
/* DBUS 接收缓冲（18 字节） */
static uint8_t g_dbus_buf[hello_world::remote_control::kRcRxDataLen];

/* 遥控离线检测移到 ins_comm.cpp，Robot 通过 GetRemoteOffline() 读取 */

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
volatile float g_dbg_g2c_vx = 0.0f;    ///< 下发的云台系 vx，m/s
volatile float g_dbg_g2c_vy = 0.0f;    ///< 下发的云台系 vy，m/s
volatile float g_dbg_g2c_spin = 0.0f;  ///< 下发的自旋角速度，rad/s
volatile uint8_t g_dbg_g2c_mode = 0;   ///< 下发的底盘模式（0/1/2）

/* 调试寄存器：CAN1 发送/错误状态 */
volatile uint32_t g_dbg_can1_tsr = 0;   ///< CAN1->TSR，发送状态
volatile uint32_t g_dbg_can1_esr = 0;   ///< CAN1->ESR，错误状态
volatile uint32_t g_dbg_can1_free = 0;  ///< CAN1 空闲发送邮箱数（0~3）

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
  auto *rc_offline = GetRemoteOffline();
  g_dbg_rc_offline =
      (rc_offline != nullptr && rc_offline->isOffline()) ? 1 : 0;

  /* ---- 云台电机：每周期 encode + 发送 ---- */
  /* 达妙上电默认失能，encode() 内部会自动发 0xFC 使能命令，
   * 收到 kMotorEnabled 反馈后才改为发 MIT 位置帧。 */
  SendGimbalMotors();

  /* ---- 双板帧：G2C（0x211），云台 → 底盘 ---- */
  auto *comm = GetGimbalChassisComm();
  if (comm != nullptr) {
    auto *imu = GetImu();
    /* delta = 云台朝向 − 车体朝向，用 yaw 电机编码器算。
     * 云台板 IMU 跟着云台动，安装倾角不固定，不能用来算 yaw。
     * 对方离线时归零：此时没有可信的车体基准。 */
    auto *gimbal = GetGimbal();
    auto *yaw_motor = GetMotorYaw();
    float delta = 0.0f;
    if (!comm->isOffline() && gimbal != nullptr && yaw_motor != nullptr) {
      delta = yaw_motor->angle() - gimbal->yawZero();
    }
    comm->gimbal_data().delta = delta;
    comm->gimbal_data().remote_ok =
        !(GetRemoteOffline() != nullptr && GetRemoteOffline()->isOffline());
    /* 自检通过 = IMU 零漂完成。不能写成“在 Working”，否则两板互相等待。 */
    if (imu != nullptr) {
      comm->gimbal_data().gimbal_ok = imu->isCalibrated();
    }
    /* 遥控状态：离线或 S2 下档急停，速度指令全 0 */
    const bool rc_lost =
        (GetRemoteOffline() != nullptr && GetRemoteOffline()->isOffline());
    const bool estop = EstopFromS2(g_dbg_rc_r_sw);

    /* 云台系速度指令：右摇杆平移（前 / 左为正）、拨轮自旋 */
    float vx = 0.0f;
    float vy = 0.0f;
    float spin = 0.0f;
    if (!rc_lost && !estop) {
      vx = kMaxLinearSpeed * ScaleStick(g_dbg_rc_rv);
      vy = -kMaxLinearSpeed * ScaleStick(g_dbg_rc_rh);
      spin = kMaxSpin * ScaleStick(g_dbg_rc_wheel);
    }
    comm->gimbal_data().vx = vx;
    comm->gimbal_data().vy = vy;
    comm->gimbal_data().spin = spin;
    comm->gimbal_data().mode = ModeFromS1(g_dbg_rc_l_sw);
    comm->gimbal_data().estop = estop;

    g_dbg_g2c_vx = vx;
    g_dbg_g2c_vy = vy;
    g_dbg_g2c_spin = spin;
    g_dbg_g2c_mode = static_cast<uint8_t>(comm->gimbal_data().mode);

    /* 云台目标：左摇杆映射，只在 Working 且未急停时接管，
     * 其余状态沿用 Robot 的归零路径 */
    auto *sys = GetRobot();
    const bool stick_on = (sys != nullptr) &&
                          (sys->lifeState() == robot::LifeState::kWorking) &&
                          !rc_lost && !estop;
    if (gimbal != nullptr) {
      gimbal->updateStick(g_dbg_rc_lv, g_dbg_rc_lh, stick_on);
    }

    uint8_t comm_buf[8] = {0};
    comm->encode(comm_buf);
    CAN_Send_Msg(GIMBAL_COMM_CAN, comm_buf, comm->tx_id(), 8);
    g_comm_tx_cnt = g_comm_tx_cnt + 1;
    g_dbg_delta = comm->gimbal_data().delta;
  }

  /* ---- 调试：CAN1 发送状态与错误状态 ---- */
  g_dbg_can1_tsr = CAN1->TSR;
  g_dbg_can1_esr = CAN1->ESR;
  g_dbg_can1_free = HAL_CAN_GetTxMailboxesFreeLevel(GIMBAL_MOTOR_CAN);

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
        auto *rc_offline = GetRemoteOffline();
        if (rc_offline != nullptr) {
          rc_offline->update();
        }
        g_dbus_rx_cnt = g_dbus_rx_cnt + 1;
      }
    }
    /* 无论是否成功，都重启下一轮接收 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_buf,
                                 hello_world::remote_control::kRcRxDataLen);
  }
}
