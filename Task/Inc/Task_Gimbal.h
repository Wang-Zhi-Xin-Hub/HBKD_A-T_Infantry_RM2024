/*!
 * @file    Task_Gimbal.h
 * @date    2024-1-1
 * @brief   云台控制任务头文件
*/

#ifndef __TASK_GIMBAL_H
#define __TASK_GIMBAL_H

#include "Variate.h"
#include "Function.h"
#include "Task_Vision.h"
/**
* @brief 云台归中
*/
void Median_Init(void);     
/**
* @brief 云台遥控器控制
*/
void Gimbal_Rc_Ctrl(void);
/**
* @brief 云台键鼠控制
*/
void Gimbal_Key_Ctrl(void);
/**
* @brief 云台电机驱动
*/
void Gimbal_Move(void);	


#endif
