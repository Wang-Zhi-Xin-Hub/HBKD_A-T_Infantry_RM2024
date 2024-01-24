/*!
 * @file    Task_Chassis_down.h
 * @date    2024-1-1
 * @brief   底盘控制（下板通信）任务头文件
 */

#ifndef __TASK_CHASSIS_DOWN_H
#define __TASK_CHASSIS_DOWN_H

#include "Variate.h"
#include "Function.h"

/**
* @brief  底盘键鼠控制
*/
void Chassis_Key_Ctrl(void);

/**
* @brief 底盘遥控器控制
*/
void Chassis_RC_Ctrl(void);

/**
* @brief 底盘跟随
*/
void Chassis_Follow(void);

/**
* @brief 变速小陀螺
*/
void Variable_Speed_Gyroscope(void);

/**
* @brief 底盘驱动
*/
void Chassis_Move(void);

/**
* @brief 发给下板当前机器人状态
*/
void Send_UI_State(void);

#endif
