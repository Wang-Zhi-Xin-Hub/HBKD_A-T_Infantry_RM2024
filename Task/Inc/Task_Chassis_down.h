#ifndef __TASK_CHASSIS_DOWN_H
#define __TASK_CHASSIS_DOWN_H

#include "Variate.h"
#include "Function.h"

void Chassis_RC_Ctrl();          //!< @brief 遥控器模式
void Chassis_Key_Ctrl();         //!< @brief 键鼠模式
void Chassis_Drive();             //!< @brief 底盘移动
void Chassis_Follow();           //!< @brief 底盘跟随
void Chassis_Offset();           //!< @brief 底盘补偿计算
void Variable_Speed_Gyroscope(); //!< @brief 变速小陀螺

#endif
