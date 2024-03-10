/*!
 * @file     Task_Vision.h
 * @date     2024-1-1
 * @auther   王志鑫
 * @brief    视觉通信（自瞄）任务头文件
 */
#ifndef __TASK_VISION_H
#define __TASK_VISION_H

#include "Variate.h"
#include "Function.h"

/**
* @brief 自瞄控制
*/
void Aim_Control(void);
/**
* @brief 给视觉发送数据
*/
void Send_to_Vision(void);
/**
* @brief 坐标转换(N->B)
*/
void Coordinate_Transformation (RotationMatrix_t R, const float* PoseN, float* PoseB);
/**
* @brief 坐标点到原点的距离
*/
float DistanceToOrigin(float X, float Y, float Z);
#endif
