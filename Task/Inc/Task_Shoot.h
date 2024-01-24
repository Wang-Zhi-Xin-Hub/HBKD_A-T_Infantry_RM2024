/*!
* @file     Task_Shoot.h
* @date     2024-1-1
* @brief    发射任务头文件
*/
#ifndef __TASK_SHOOT_H
#define __TASK_SHOOT_H

#include "Variate.h"
#include "Function.h"
/**
 * @brief 发射机构遥控器控制
 */
void Shoot_Rc_Ctrl(void);

/**
 * @brief 发射机构键鼠模式
 */
void Shoot_Key_Ctrl(void);

/**
 * @brief发射机构电机驱动
 */
void Shoot_Move(void);

#endif
