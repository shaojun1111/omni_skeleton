/**
 *******************************************************************************
 * @file      :gimbal.hpp
 * @brief     :云台运动模块（yaw / pitch 两轴位置环）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-08      lishaojun       1. 初版：角度差 → PID → MIT 力矩
 *  V1.1.0      2026-11-11      lishaojun       2. 遥控摇杆接管 yaw / pitch 目标
 *  V1.2.0      2026-11-14      lishaojun       3. 小陀螺：yaw 改用 IMU 世界角闭环
 *******************************************************************************
 * @attention :
 *  1. yaw 连续旋转，目标角在 update() 里投影到当前圈再交给 PID。
 *  2. pitch 在 setTarget() 里做软件限位，防止顶结构。
 *  3. update() 每 1ms 调用一次。
 *  4. 跟随 / 小陀螺模式下 yaw 反馈从关节角换成 IMU 世界角（除以 k 折算成关节
 *     等效角），底盘自转时枪口保持指向；这也让 δ 能随底盘转动而变、跟随环才闭合。
 *     停车 / 回到分离模式时切回关节角。
 *  5. updateFeedback() 只读反馈、算 δ，任何生命状态都可每周期调用；
 *     电机输出只由 update() / stop() 决定，失效归零不影响零位和 k 的标定。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_ROBOT_MODULES_GIMBAL_HPP_
#define GIMBAL_ROBOT_MODULES_GIMBAL_HPP_

/* Includes ------------------------------------------------------------------*/
#include "module_state.hpp"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace robot
{

class Gimbal
{
 public:
  /**
   * @brief 初始化：目标角设为当前反馈角、复位 PID、输出 0 力矩
   */
  void init(void);

  /**
   * @brief 下发目标关节角
   * @param target yaw / pitch，单位 rad
   * @note  pitch 在内部钳到 [kPitchMin, kPitchMax]
   */
  void setTarget(const GimbalState& target);

  /**
   * @brief 遥控摇杆映射：yaw 速度积分、pitch 绝对角
   * @param lv 左摇杆竖直，[-1, 1]，前推为正
   * @param lh 左摇杆水平，[-1, 1]，右推为正
   * @param enabled true：目标由摇杆决定；false：回到 setTarget() 的目标
   * @note  每 1ms 调用一次；yaw 松杆停在当前目标，不回正
   */
  void updateStick(float lv, float lh, bool enabled);

  /**
   * @brief 世界角闭环开关：打开后 yaw 改用 IMU 世界角闭环，枪口在世界上保持指向
   * @param on true = 跟随 / 小陀螺（底盘会自己转）；false = 回到关节角闭环
   * @note  每 1ms 调用一次。IMU 未标定或 k 未锁定时不生效（继续用关节角）。
   *        切换时目标会重新对齐到新的反馈量，不会跳变。
   */
  void setWorldRef(bool on);

  /**
   * @brief 每周期测量：读 yaw 反馈、第一帧锁零位、自标定 k、更新 δ
   * @note  不写电机、不跑控制环，Dead / Resurrection 也可以调用。
   *        零位和 k 不依赖生命状态，失效归零只停输出，不改变前进方向。
   */
  void updateFeedback(void);

  /**
   * @brief 周期任务：算误差 → PID → setMitInput
   * @note  每 1ms 调用一次；内部先调 updateFeedback() 刷新反馈与 δ
   */
  void update(void);

  /**
   * @brief 立即停止：输出 0 力矩、PID 复位、目标回到当前角
   */
  void stop(void);

  /* 调试用：最近一次反馈与输出 */
  float yawFeedback(void) const { return yaw_fb_; }
  float pitchFeedback(void) const { return pitch_fb_; }
  float yawOutput(void) const { return yaw_out_; }
  float pitchOutput(void) const { return pitch_out_; }

  /**
   * @brief 云台相对车头的偏航角 δ（逆时针为正），底盘用它把云台系速度转到车体系
   * @note  δ = k * (yaw_joint - yawZero)，k 的符号/量程由 IMU 自学习得到
   */
  float yawDelta(void) const { return delta_; }

  /**
   * @brief δ 是否可用：关节零位已记录，且 k 的符号已用 IMU 标定
   * @note  为假时底盘按 δ = 0 处理（车体系平移），避免用错符号把方向搞反
   */
  bool deltaValid(void) const { return yaw_zero_valid_ && sign_latched_; }

  /* 调试：自学习到的 k（IMU yaw 增量 / 关节角增量），未标定时返回 0 */
  float yawJointScale(void) const { return sign_latched_ ? yaw_scale_ : 0.0f; }

  /* 调试：世界角闭环是否生效 */
  bool worldRefActive(void) const { return world_ref_; }

 private:
  /**
   * @brief 用「关节在动、IMU yaw 也在动」的样本回归出 k = Δimu_yaw / Δjoint
   * @note  只在关节有明显转动时累计；R² 不够高（底盘同时在转）就放弃本次样本
   */
  void updateYawScale(float yaw_joint);

  /**
   * @brief yaw 环当前用的反馈角（关节等效角）
   * @return 关节角；小陀螺模式且 k 已锁定时返回 IMU yaw / k
   * @note  imu_yaw = k * 关节角 + 常数，除以 k 后与关节角同量纲，PID 参数不用改
   */
  float yawLoopFeedback(void) const;

  GimbalState target_{};
  float yaw_fb_ = 0.0f;     ///< yaw 反馈角，rad
  float pitch_fb_ = 0.0f;   ///< pitch 反馈角，rad
  float yaw_out_ = 0.0f;    ///< yaw 输出力矩，N·m
  float pitch_out_ = 0.0f;  ///< pitch 输出力矩，N·m
  float yaw_zero_ = 0.0f;   ///< 云台朝向零位，rad（第一帧反馈时记录）
  bool yaw_zero_valid_ = false;  ///< 零位是否已记录
  float delta_ = 0.0f;      ///< 最近一次算出的 δ，rad

  float yaw_scale_ = -1.0f;     ///< 关节角 → 世界 yaw 的比例（含符号）
  bool sign_latched_ = false;   ///< yaw_scale_ 是否已由 IMU 标定锁定
  float sum_jj_ = 0.0f;         ///< Σ Δjoint²
  float sum_jy_ = 0.0f;         ///< Σ Δjoint·Δimu_yaw
  float sum_yy_ = 0.0f;         ///< Σ Δimu_yaw²
  float sum_travel_ = 0.0f;     ///< Σ |Δjoint|，累计关节行程（rad）
  float last_joint_ = 0.0f;     ///< 上一周期关节角
  float last_imu_yaw_ = 0.0f;   ///< 上一周期 IMU yaw

  float stick_yaw_ = 0.0f;    ///< 摇杆积分出的 yaw 目标，rad（小陀螺下为关节等效角）
  float stick_pitch_ = 0.0f;  ///< 摇杆给出的 pitch 目标，rad
  bool stick_active_ = false;  ///< 摇杆是否接管目标
  bool world_ref_ = false;     ///< 世界角闭环是否生效
};

}  // namespace robot

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif /* GIMBAL_ROBOT_MODULES_GIMBAL_HPP_ */
