/**
 *******************************************************************************
 * @file      : imu.hpp
 * @brief     : IMU 设备封装（BMI088 + Mahony + 开机零漂）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-12-01      Jinletian       1. 初版编写完成
 *  V2.0.0      2026-09-18      Hello World     1. 考核骨架：掏空 Imu 封装类
 *  V2.1.0      2026-10-28      lishaojun       1. 实现 Imu：零漂标定 + 姿态解算
 *******************************************************************************
 * @attention :
 *  1. 姿态坐标系：x 朝前，y 朝左，z 朝上（见 ahrs_base.hpp）。
 *  2. update() 每 1ms 调用一次，对应 Mahony 的 samp_freq = 1000。
 *  3. 上电后静止 1000 个周期（约 1s）完成零漂标定，期间不更新姿态。
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_IMU_IMU_HPP_
#define HW_COMPONENTS_DEVICES_IMU_IMU_HPP_

/* Includes ------------------------------------------------------------------*/
#include "BMI088.hpp"
#include "mahony.hpp"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

namespace hello_world
{
namespace imu
{

class Imu
{
 public:
  Imu(void) = default;

  /**
   * @brief 初始化 BMI088 与 Mahony，并复位零漂标定状态
   * @note  内部含阻塞式 SPI 配置，只能在 MainTaskInit 里调用
   */
  void init(void);

  /**
   * @brief 读芯片、扣零漂、更新姿态
   * @note  每 1ms 调用一次
   */
  void update(void);

  /* 姿态角，单位 rad */
  float roll(void) const { return euler_[0]; }
  float pitch(void) const { return euler_[1]; }
  float yaw(void) const { return euler_[2]; }

  /* 零漂标定是否完成 */
  bool isCalibrated(void) const { return calibrated_; }

  /* BMI088 初始化错误位，0 表示无错误 */
  uint8_t errState(void) const { return static_cast<uint8_t>(err_); }

  /* 当前零漂估计值，单位 rad/s，调试用 */
  float gyroBias(int axis) const
  {
    return (axis >= 0 && axis < 3) ? gyro_bias_[axis] : 0.0f;
  }

 private:
  static constexpr uint32_t kCaliSamples = 1000;  ///< 零漂采样次数，1ms × 1000 = 1s

  BMI088 bmi088_;
  ahrs::Mahony mahony_;

  float acc_[3] = {0.0f, 0.0f, 0.0f};        ///< 加速度，m/s²
  float gyro_[3] = {0.0f, 0.0f, 0.0f};       ///< 角速度，rad/s
  float gyro_bias_[3] = {0.0f, 0.0f, 0.0f};  ///< 零漂估计，rad/s
  float euler_[3] = {0.0f, 0.0f, 0.0f};      ///< [roll, pitch, yaw]，rad

  uint32_t cali_cnt_ = 0;   ///< 已采样次数
  bool calibrated_ = false; ///< 标定完成标志
  BMI088ErrState err_ = kBMI088ErrStateNoErr;
};

}  // namespace imu
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_IMU_IMU_HPP_ */
