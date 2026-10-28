/**
 *******************************************************************************
 * @file      : imu.cpp
 * @brief     : IMU 封装实现（BMI088 + Mahony + 开机零漂）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-28      lishaojun       1. 实现 init/update 与零漂标定
 *******************************************************************************
 * @attention :
 *  1. BMI088：SPI1，加速度计片选 PA4，陀螺仪片选 PB0。
 *  2. 零漂标定需要静止 1000 个周期（约 1s），期间 euler_ 保持 0。
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "imu.hpp"

#include "main.h"
#include "spi.h"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace
{

/* BMI088 安装方向与车体系一致时的旋转矩阵（单位矩阵） */
const float kRotIdentity[9] = {1.0f, 0.0f, 0.0f,
                               0.0f, 1.0f, 0.0f,
                               0.0f, 0.0f, 1.0f};

/* Mahony 参数：采样率 1000Hz，比例增益 1.0，积分增益 0.0 */
constexpr float kMahonyFreq = 1000.0f;
constexpr float kMahonyKp = 1.0f;
constexpr float kMahonyKi = 0.0f;

}  // namespace

/* Exported function definitions ---------------------------------------------*/
namespace hello_world
{
namespace imu
{

void Imu::init(void)
{
  /* 1. 硬件配置：SPI1 + 两个片选 */
  BMI088HWConfig hw;
  hw.hspi = &hspi1;
  hw.acc_cs_port = GPIOA;
  hw.acc_cs_pin = GPIO_PIN_4;
  hw.gyro_cs_port = GPIOB;
  hw.gyro_cs_pin = GPIO_PIN_0;

  /* 2. 保存配置并写芯片寄存器（阻塞几毫秒） */
  bmi088_.init(hw, kRotIdentity, BMI088Config());
  err_ = bmi088_.imuInit(false);

  /* 3. 姿态解算器初始化 */
  mahony_.init(kMahonyFreq, kMahonyKp, kMahonyKi);

  /* 4. 复位零漂标定状态 */
  cali_cnt_ = 0;
  calibrated_ = false;
  for (int i = 0; i < 3; ++i) {
    gyro_bias_[i] = 0.0f;
    euler_[i] = 0.0f;
  }
}

void Imu::update(void)
{
  /* 读一次芯片：加速度 m/s²，角速度 rad/s */
  bmi088_.getData(acc_, gyro_, nullptr);

  if (!calibrated_) {
    /* ---- 标定阶段：累加求均值 ---- */
    for (int i = 0; i < 3; ++i) {
      gyro_bias_[i] += gyro_[i];
    }
    cali_cnt_ += 1;
    if (cali_cnt_ >= kCaliSamples) {
      for (int i = 0; i < 3; ++i) {
        gyro_bias_[i] /= static_cast<float>(kCaliSamples);
      }
      calibrated_ = true;
    }
    return;
  }

  /* ---- 运行阶段：扣零漂后解算姿态 ---- */
  float gyro_corrected[3];
  for (int i = 0; i < 3; ++i) {
    gyro_corrected[i] = gyro_[i] - gyro_bias_[i];
  }

  mahony_.update(acc_, gyro_corrected);
  mahony_.getEulerAngle(euler_);
}

}  // namespace imu
}  // namespace hello_world
