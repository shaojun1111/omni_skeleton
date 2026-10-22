/**
 *******************************************************************************
 * @file      :module_state.hpp
 * @brief     :底盘板与云台板共享的状态类型与枚举
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-18      lishaojun       1. 初版
 *******************************************************************************
 * @attention :
 *  1. 本文件被两板同时编译，禁止 include 任何硬件头（motor/CAN/HAL）。
 *  2. 单位约定：速度 m/s，角速度 rad/s，角度 rad。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MODULE_STATE_HPP_
#define MODULE_STATE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace robot{

// 1) 生命状态
enum class LifeState : uint8_t {
    kDead,
    kResurrection,
    kWorking,
};

// 2）手动控制源
enum class ControlSource : uint8_t {
    kNone = 0,
    kRemoteControl,
};

// 3) 底盘工作模式
enum class ChassisMode : uint8_t {
    kSeparate = 0,
    kFollow,
    kSpin,
};

// 4) 云台工作模式
enum class GimbalMode : uint8_t {
    kJoint = 0,
};

// 5) 底盘运动状态
struct ChassisState {
    float vx = 0.0f;  ///< 平移速度，云台系 x 轴（前方），单位：m/s
    float vy = 0.0f;  ///< 平移速度，云台系 y 轴（左方），单位：m/s
    float w  = 0.0f;  ///< 旋转角速度，逆时针为正，单位：rad/s
};

// 6) 云台运动状态
struct GimbalState {
    float yaw   = 0.0f;  ///< 偏航关节角，单位：rad
    float pitch = 0.0f;  ///< 俯仰关节角，单位：rad
};

// 7)运算
inline ChassisState operator+(const ChassisState& a, const ChassisState& b) {
    return ChassisState{a.vx + b.vx, a.vy + b.vy, a.w + b.w};
}
inline ChassisState operator-(const ChassisState& a, const ChassisState& b) {
    return ChassisState{a.vx - b.vx, a.vy - b.vy, a.w - b.w};
}
inline ChassisState operator*(float k, const ChassisState& a) {
    return ChassisState{k * a.vx, k * a.vy, k * a.w};
}

} // namespace robot
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/


#endif /* MODULE_STATE_HPP_ */
