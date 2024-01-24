/*!
 * @file     Task_Protect.c
 * @date     2024-1-1
 * @brief    断控保护（看门狗）任务
 */
#include "Task_Protect.h"

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
			Chassis_Close();			   // 底盘关
			Gimbal_Close();				   // 云台关
			Shoot_Close();				   // 射击机构关
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
        
		vTaskDelayUntil(&currentTime, 2); // 绝对延时2ms
	}
}
