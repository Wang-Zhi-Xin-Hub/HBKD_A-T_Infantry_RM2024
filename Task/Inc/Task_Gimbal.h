#ifndef __TASK_GIMBAL_H
#define __TASK_GIMBAL_H

#include "Variate.h"
#include "Function.h"
#include "Task_Vision.h"

void Median_Init();      //!< @brief 云台归中
void Gimbal_Rc_Ctrl();   //!< @brief 遥控器模式
void Gimbal_Key_Ctrl();  //!< @brief 键鼠模式
void Gimbal_Drive();     //!< @brief 云台PID计算函数

#endif
