/*!
 * @file     Function.h
 * @brief    声明全局调用功能函数头文件
 */

#ifndef __FUNCTION_H
#define __FUNCTION_H

#include "Variate.h"

/**@brief 求绝对值 */
#define ABS(x) ((x) > 0 ? (x) : (-(x)))

/**@brief 去除遥控器摇杆死区 */
#define deadline_limit(value, dealine)                     \
	{                                                      \
		if ((value) <= (dealine) && (value) >= -(dealine)) \
			value = 0;                                     \
	}

/**@brief 云台电机关闭函数 */
void Gimbal_Close(void);

/**@brief 云台电机急停函数 */
void Gimbal_Stop(void);

/**@brief 发射机构电机关闭函数 */
void Shoot_Close(void);

/**@brief 发射机构电机急停函数 */
void Shoot_Stop(void);

/**@brief 底盘电机关闭函数 */
void Chassis_Close(void);

/**@brief 底盘电机急停函数 */
void Chassis_Stop(void);

/**@brief 斜坡函数 */
float RAMP_float(float final, float now, float ramp);

/**@brief 3508过温检测函数 */
osStatus_t RM3508_Motor_Temp(RM3508_TypeDef *dst);

/**@brief 6020过温检测函数 */
osStatus_t GM6020_Motor_Temp(GM6020_TypeDef *dst);

/**@brief 遥控器数据乱码检测 */
osStatus_t REMOTE_IfDataError(void);

/**@brief 陀螺仪数据乱码检测 */
osStatus_t IMU_IfDataError(void);

#endif
