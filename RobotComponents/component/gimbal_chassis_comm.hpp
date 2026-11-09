/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.hpp
 * @brief     :双板 CAN 通讯协议（底盘 <-> 云台）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2026-10-22      lishaojun       1. 骨架初版
 *  V1.0.0      2026-11-09      lishaojun       1. 定字段：G2C 控制指令 + C2G 状态
 *******************************************************************************
 * @attention :
 *  1. 帧格式见 lessons_opencode/11_board_comm.md 第 4 节，两板必须逐字节一致。
 *  2. 小端字节序（低字节在前）。
 *  3. 一个类服务两块板，code_part_ 决定 encode / decode 的方向。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_CHASSIS_COMM_HPP_
#define GIMBAL_CHASSIS_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "module_state.hpp"
#include "tick.hpp"

/* Exported macro ------------------------------------------------------------*/
namespace robot {
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* 底盘↔云台双板 CAN 通讯协议类。底盘板与云台板各实例化一个，用 code_part 区分方向。 */
class GimbalChassisComm {
 public:
  /* 对方超过这个时长没有新帧，判定离线（ms） */
  static constexpr uint32_t kOfflineThresholdMs = 100;

  /* 标记本实例运行在哪块板，决定 encode / decode 的方向。 */
  enum class CodePart : uint8_t { Chassis = 0, Gimbal = 1 };

  /* 底盘→云台（C2G，0x210）帧数据。 */
  struct ChassisData {
    float chassis_yaw = 0.0f;                  ///< 底盘 IMU yaw，rad
    LifeState life_state = LifeState::kDead;   ///< 底盘生命状态
    bool chassis_ok = false;                   ///< 底盘已初始化
    bool imu_ok = false;                       ///< 底盘 IMU 零漂完成
  };

  /* 云台→底盘（G2C，0x211）帧数据。 */
  struct GimbalData {
    float vx = 0.0f;      ///< 平移速度，云台系 x 轴，m/s
    float vy = 0.0f;      ///< 平移速度，云台系 y 轴，m/s
    float spin = 0.0f;    ///< 自旋角速度，逆时针为正，rad/s
    float delta = 0.0f;   ///< 云台朝向 − 车体朝向，rad
    ChassisMode mode = ChassisMode::kSeparate;  ///< 底盘工作模式
    bool remote_ok = false;                     ///< 遥控在线
    bool estop = false;                         ///< 急停请求
    bool gimbal_ok = false;                     ///< 云台已初始化
  };

  GimbalChassisComm(CodePart code_part, uint32_t tx_id, uint32_t rx_id);

  /* 编码己方帧：把本地 ChassisData / GimbalData 打包进 8 字节。 */
  void encode(uint8_t tx_data[8]);
  /* 解码对方帧：把 8 字节还原成对方语义并更新本地缓存。 */
  void decode(const uint8_t rx_data[8]);

  /* 判断对方是否离线（超过 kvOfflineThresholdMs 未收到新帧）。 */
  bool isOffline() const;

  /* 本板发送帧 ID / 接收帧 ID（供 CommTask 发送时取用）。 */
  uint32_t tx_id() const { return tx_id_; }
  uint32_t rx_id() const { return rx_id_; }

  /* 已收到的对方帧数（调试用）。 */
  uint32_t rxCount() const { return rx_count_; }

  /* 数据访问（供 RobotModules / Task 读写）。 */
  ChassisData &chassis_data() { return chassis_data_; }
  GimbalData &gimbal_data() { return gimbal_data_; }
  const ChassisData &chassis_data() const { return chassis_data_; }
  const GimbalData &gimbal_data() const { return gimbal_data_; }

 private:
  CodePart code_part_;
  uint32_t tx_id_;
  uint32_t rx_id_;
  uint32_t last_rx_tick_;  ///< 上次收到对方帧的 GetTickMs() 时间戳
  uint32_t rx_count_ = 0;  ///< 收到对方帧的累计数
  ChassisData chassis_data_;
  GimbalData gimbal_data_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot

#endif /* GIMBAL_CHASSIS_COMM_HPP_ */
