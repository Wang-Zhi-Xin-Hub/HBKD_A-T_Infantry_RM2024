/*!
 * @file     Task_Protect.c
 * @date     2024-1-1
 * @brief    断控保护（看门狗）任务
 */
#include "Task_Protect.h"
#define LOOK_STACK 0

#if LOOK_STACK
UBaseType_t uxHighWaterMark1,uxHighWaterMark2,uxHighWaterMark3,uxHighWaterMark4,uxHighWaterMark5,uxHighWaterMark6;
#endif

/* 断控保护任务（!安全第一） */
void Task_Protect(void *pvParameters)
{
	static portTickType currentTime;
	for (;;)
	{
		currentTime = xTaskGetTickCount(); // 获取当前系统时间

		if (Remote_State != Device_Online)
		{
			osThreadSuspend(Task_Chassis_down_handle); // 将任务挂起
			osThreadSuspend(Task_Shoot_handle);
			osThreadSuspend(Task_Gimbal_handle);

			systemState = SYSTEM_STARTING; // 系统恢复至重启状态
			RemoteClear();				   // 遥控数据恢复至默认状态
			Chassis_Close();			   // 底盘电机关闭
			Gimbal_Close();				   // 云台电机关闭
			Shoot_Close();				   // 发射机构电机关闭
		}
		else
		{
			/* 恢复任务 */
			osThreadResume(Task_Chassis_down_handle);
			osThreadResume(Task_Shoot_handle);
			osThreadResume(Task_Gimbal_handle);
		}

		/* 看门狗轮询 */
		WatchDog_Polling();
        
#if LOOK_STACK
        /* 读取任务剩余Stack，防止溢出 */
        uxHighWaterMark1 = uxTaskGetStackHighWaterMark( Task_Chassis_down_handle );
        uxHighWaterMark2 = uxTaskGetStackHighWaterMark( Task_Gimbal_handle );
        uxHighWaterMark3 = uxTaskGetStackHighWaterMark( Task_Shoot_handle );
        uxHighWaterMark4 = uxTaskGetStackHighWaterMark( Task_Protect_handle );
        uxHighWaterMark5 = uxTaskGetStackHighWaterMark( Task_IMU_handle );
        uxHighWaterMark6 = uxTaskGetStackHighWaterMark( Task_Remote_handle );
#endif
        
		vTaskDelayUntil(&currentTime, 10); // 绝对延时10ms
	}
}
