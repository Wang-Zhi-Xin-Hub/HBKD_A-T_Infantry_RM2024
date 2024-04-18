#include "Task_Protect.h"

#define LOOK_STACK 0
#if LOOK_STACK
UBaseType_t uxHighWaterMark[6];//观察任务堆栈使用情况
char taskListBuffer[30*6];//保存任务运行时间信息 分别是：任务名 任务状态 优先级 剩余栈 任务序号
char taskStateBuffer[30*6];//保存任务运行时间信息 分别是：任务名 运行计数  使用率
#endif

/* 断控保护任务（!安全第一） */
void Task_Protect(void *pvParameters)
{
	static portTickType currentTime;
	for (;;)
	{
		currentTime = xTaskGetTickCount(); // 获取当前系统时间

		if (Remote_State != Device_Online){
			osThreadSuspend(Task_Chassis_down_handle); // 将任务挂起
			osThreadSuspend(Task_Shoot_handle);
			osThreadSuspend(Task_Gimbal_handle);

			systemState = SYSTEM_STARTING; // 系统恢复至重启状态
			RemoteClear();				   // 遥控数据恢复至默认状态
			Chassis_Close();			   // 底盘电机关闭
			Gimbal_Close();				   // 云台电机关闭
			Shoot_Close();				   // 发射机构电机关闭
		} else{
			/* 恢复任务 */
			osThreadResume(Task_Chassis_down_handle);
			osThreadResume(Task_Shoot_handle);
			osThreadResume(Task_Gimbal_handle);
		}
		WatchDog_Polling(); ///软件看门狗轮询
        HAL_IWDG_Refresh(&hiwdg);//硬件看门狗
        Send_UI_State();    //给下板发送状态

#if LOOK_STACK
        //获得任务名 任务状态 优先级 剩余栈 任务序号
        memset(taskListBuffer, 0, 30*6);
        vTaskList((char *)&taskListBuffer); 
        //获取剩余Stack大小,堆栈不够会进入硬件错误
        uxHighWaterMark[0] = uxTaskGetStackHighWaterMark( Task_Chassis_down_handle );
        uxHighWaterMark[1] = uxTaskGetStackHighWaterMark( Task_Gimbal_handle );
        uxHighWaterMark[2] = uxTaskGetStackHighWaterMark( Task_Shoot_handle );
        uxHighWaterMark[3] = uxTaskGetStackHighWaterMark( Task_Protect_handle );
        uxHighWaterMark[4] = uxTaskGetStackHighWaterMark( Task_IMU_handle );
        uxHighWaterMark[5] = uxTaskGetStackHighWaterMark( Task_Remote_handle );
        //获取任务名 运行计数  使用率
        memset(taskStateBuffer, 0, 30*6);
        vTaskGetRunTimeStats((char *)&taskStateBuffer);
#endif
		vTaskDelayUntil(&currentTime, 10);
	}
}

/* 发送机构状态 */
void Send_UI_State()
{
    Communication_Action_Tx.ChassisAction_Tx = ChassisAction;
    Communication_Action_Tx.ShootAction_Tx = ShootAction;
    Communication_Action_Tx.AimAction_Tx = AimAction;
    Communication_Action_Tx.CtrlMode_Tx = CtrlMode;
    Communication_Action_Tx.MidMode_Tx = MidMode;

    CAN_Send_StdDataFrame(&hcan2, 0x120, (uint8_t *)&Communication_Action_Tx);
}
