/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.cpp
 * @brief     :双板 CAN 通讯协议实现（编码 / 解码 / 离线判断）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2026-10-22      lishaojun       1. 骨架初版
 *  V1.0.0      2026-11-09      lishaojun       1. encode/decode + 小端字节序
 *******************************************************************************
 * @attention :
 *  1. 浮点一律先钳位再转整数：static_cast 到 int16 时超出范围是未定义行为。
 *  2. 字节序统一小端（低字节在前），收发两端一致。
 *  3. decode() 写"对方"的结构体，并刷新 last_rx_tick_ 与 rx_count_。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "gimbal_chassis_comm.hpp"

#include <cmath>

#include "base.hpp"  // AngleNormRad()

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace
{

/* 量化比例（编码时乘，解码时除） */
constexpr float kVxScale = 1000.0f;     ///< 1e-3 m/s
constexpr float kVyScale = 1000.0f;     ///< 1e-3 m/s
constexpr float kDeltaScale = 10000.0f; ///< 1e-4 rad
constexpr float kSpinScale = 20.0f;     ///< 0.05 rad/s → 乘 20
constexpr float kStickScale = 100.0f;   ///< 摇杆 1/100

/* 浮点 → int16：先钳位到对称范围，再四舍五入 */
int16_t ToI16(float x)
{
  if (x > 32767.0f) {
    x = 32767.0f;
  }
  if (x < -32767.0f) {
    x = -32767.0f;
  }
  return static_cast<int16_t>(roundf(x));
}

/* 浮点 → int8：先钳位到对称范围，再四舍五入 */
int8_t ToI8(float x)
{
  if (x > 127.0f) {
    x = 127.0f;
  }
  if (x < -127.0f) {
    x = -127.0f;
  }
  return static_cast<int8_t>(roundf(x));
}

/* int16 → 两字节（小端） */
void PutI16(uint8_t* p, int16_t v)
{
  const uint16_t u = static_cast<uint16_t>(v);
  p[0] = static_cast<uint8_t>(u & 0xFFu);
  p[1] = static_cast<uint8_t>((u >> 8) & 0xFFu);
}

/* 两字节 → int16（小端）：先拼 uint16 再转，负数补码自动还原 */
int16_t GetI16(const uint8_t* p)
{
  const uint16_t u =
      static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
  return static_cast<int16_t>(u);
}

}  // namespace

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm::GimbalChassisComm(
    robot::GimbalChassisComm::CodePart code_part, uint32_t tx_id,
    uint32_t rx_id)
{
  code_part_ = code_part;
  tx_id_ = tx_id;
  rx_id_ = rx_id;
  last_rx_tick_ = hello_world::tick::GetTickMs();
}

void robot::GimbalChassisComm::encode(uint8_t tx_data[8])
{
  if (tx_data == nullptr) {
    return;
  }
  for (int i = 0; i < 8; ++i) {
    tx_data[i] = 0;
  }

  if (code_part_ == CodePart::Chassis) {
    /* C2G（0x210）：左/右摇杆 + life_state + flags + 拨轮 + 拨杆 */
    tx_data[0] = static_cast<uint8_t>(ToI8(chassis_data_.lv * kStickScale));
    tx_data[1] = static_cast<uint8_t>(ToI8(chassis_data_.lh * kStickScale));
    tx_data[2] = static_cast<uint8_t>(chassis_data_.life_state);

    uint8_t flags = 0;
    if (chassis_data_.chassis_ok) {
      flags |= 0x01u;
    }
    if (chassis_data_.imu_ok) {
      flags |= 0x02u;
    }
    if (chassis_data_.remote_ok) {
      flags |= 0x04u;
    }
    tx_data[3] = flags;

    tx_data[4] = static_cast<uint8_t>(ToI8(chassis_data_.rv * kStickScale));
    tx_data[5] = static_cast<uint8_t>(ToI8(chassis_data_.rh * kStickScale));
    tx_data[6] = static_cast<uint8_t>(ToI8(chassis_data_.wheel * kStickScale));
    tx_data[7] = static_cast<uint8_t>((chassis_data_.s1 & 0x0Fu) |
                                      ((chassis_data_.s2 & 0x0Fu) << 4));
  } else {
    /* G2C（0x211）：vx / vy / delta / spin / flags */
    PutI16(&tx_data[0], ToI16(gimbal_data_.vx * kVxScale));
    PutI16(&tx_data[2], ToI16(gimbal_data_.vy * kVyScale));
    /* delta 归一化到 [-π, π)，避免超量程被钳位 */
    PutI16(&tx_data[4],
           ToI16(hello_world::AngleNormRad(gimbal_data_.delta) * kDeltaScale));
    tx_data[6] = static_cast<uint8_t>(ToI8(gimbal_data_.spin * kSpinScale));

    uint8_t flags = 0;
    if (gimbal_data_.remote_ok) {
      flags |= 0x01u;
    }
    if (gimbal_data_.estop) {
      flags |= 0x02u;
    }
    if (gimbal_data_.gimbal_ok) {
      flags |= 0x04u;
    }
    if (gimbal_data_.delta_valid) {
      flags |= 0x20u;
    }
    flags |= static_cast<uint8_t>(
        (static_cast<uint8_t>(gimbal_data_.mode) & 0x03u) << 3);
    tx_data[7] = flags;
  }
}

void robot::GimbalChassisComm::decode(const uint8_t rx_data[8])
{
  if (rx_data == nullptr) {
    return;
  }

  if (code_part_ == CodePart::Chassis) {
    /* 底盘板收到 G2C（0x211）→ 写 gimbal_data_ */
    gimbal_data_.vx = static_cast<float>(GetI16(&rx_data[0])) / kVxScale;
    gimbal_data_.vy = static_cast<float>(GetI16(&rx_data[2])) / kVyScale;
    gimbal_data_.delta = static_cast<float>(GetI16(&rx_data[4])) / kDeltaScale;
    gimbal_data_.spin =
        static_cast<float>(static_cast<int8_t>(rx_data[6])) / kSpinScale;

    const uint8_t flags = rx_data[7];
    gimbal_data_.remote_ok = (flags & 0x01u) != 0;
    gimbal_data_.estop = (flags & 0x02u) != 0;
    gimbal_data_.gimbal_ok = (flags & 0x04u) != 0;
    gimbal_data_.delta_valid = (flags & 0x20u) != 0;
    gimbal_data_.mode =
        static_cast<ChassisMode>(static_cast<uint8_t>((flags >> 3) & 0x03u));
  } else {
    /* 云台板收到 C2G（0x210）→ 写 chassis_data_ */
    chassis_data_.lv =
        static_cast<float>(static_cast<int8_t>(rx_data[0])) / kStickScale;
    chassis_data_.lh =
        static_cast<float>(static_cast<int8_t>(rx_data[1])) / kStickScale;
    chassis_data_.life_state = static_cast<LifeState>(rx_data[2]);

    const uint8_t flags = rx_data[3];
    chassis_data_.chassis_ok = (flags & 0x01u) != 0;
    chassis_data_.imu_ok = (flags & 0x02u) != 0;
    chassis_data_.remote_ok = (flags & 0x04u) != 0;

    chassis_data_.rv =
        static_cast<float>(static_cast<int8_t>(rx_data[4])) / kStickScale;
    chassis_data_.rh =
        static_cast<float>(static_cast<int8_t>(rx_data[5])) / kStickScale;
    chassis_data_.wheel =
        static_cast<float>(static_cast<int8_t>(rx_data[6])) / kStickScale;
    chassis_data_.s1 = static_cast<uint8_t>(rx_data[7] & 0x0Fu);
    chassis_data_.s2 = static_cast<uint8_t>((rx_data[7] >> 4) & 0x0Fu);
  }

  last_rx_tick_ = hello_world::tick::GetTickMs();
  rx_count_ = rx_count_ + 1;
}

bool robot::GimbalChassisComm::isOffline() const
{
  const uint32_t now = hello_world::tick::GetTickMs();
  /* uint32 减法自动回绕，tick 溢出也正确 */
  return (now - last_rx_tick_) > kOfflineThresholdMs;
}

/* Private function definitions ----------------------------------------------*/
