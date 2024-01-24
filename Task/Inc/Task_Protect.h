/*!
* @file     Task_Protect.h
* @date     2024-1-1
* @brief    断控保护（看门狗）任务头文件
*/

#ifndef __TASK_PROTECT_H
#define __TASK_PROTECT_H

#include "Variate.h"
#include "Function.h"

extern TaskHandle_t  Task_Chassis_down_handle;
extern TaskHandle_t  Task_Gimbal_handle;
extern TaskHandle_t  Task_Shoot_handle;
extern TaskHandle_t  Task_Protect_handle;
extern TaskHandle_t  Task_IMU_handle;
extern TaskHandle_t  Task_Remote_handle;

#endif
