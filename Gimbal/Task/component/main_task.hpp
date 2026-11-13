/**
 *******************************************************************************
 * @file      :main_task.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_TASK_MAIN_TASK_HPP_
#define GIMBAL_TASK_MAIN_TASK_HPP_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Exported function prototypes ----------------------------------------------*/
void MainTaskInit(void);
void MainTask(void);
#ifdef __cplusplus
}
#endif

/* 主任务心跳：每个 1ms 周期 +1，CommTask 用它确认 MainTask 跑过再喂狗 */
extern volatile uint32_t g_main_task_beat;

#endif /* GIMBAL_TASK_MAIN_TASK_HPP_ */
