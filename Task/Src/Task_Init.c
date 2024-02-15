/*!
 * @file     Task_Init.c
 * @date     2024-1-1
 * @brief    初始化任务
 * @note     Task_Init.h中有代码框架说明以及机器人外设说明
 */
#include "Task_Init.h"

/* 初始化主任务 */
void Task_Init()
{
    taskENTER_CRITICAL(); // 进入临界区
    
    /* 看门狗初始化（初始化顺序则为看门狗ID顺序） */
    WatchDog_Init(&Remote_Dog, 40);
    WatchDog_Init(&IMU_Dog, 20);
    WatchDog_Init(&Gimbal_Dog[YAW], 15);
    WatchDog_Init(&Gimbal_Dog[PITCH], 15);
    WatchDog_Init(&Shoot_Dog[LEFT], 15);
    WatchDog_Init(&Shoot_Dog[RIGHT], 15);
    WatchDog_Init(&Pluck_Dog, 15);
    WatchDog_Init(&Down_Dog, 40);
    WatchDog_Init(&PC_Dog, 100);
    
    /* CAN1初始化(上下板通信（速度，状态、裁判系统）、拨弹盘电机2006、两个摩擦轮电机3508) */
    CanFilter_Init(&hcan1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    /* CAN2初始化(Yaw轴6020、Pitch轴电机6020) */
    CanFilter_Init(&hcan2);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
    
    /* 串口1初始化（NDJ6遥控器） */
    HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, Usart1_Remote_Dma[0], Remote_Usart1_Len);
    
    /* 串口2初始化（WT931陀螺仪） */
    HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2, Usart2_IMU_Dma[0], IMU_Usart2_Len);
       
    /* 创建任务 */
    xTaskCreate((TaskFunction_t)Task_Chassis_down, "Task_Chassis_down", 256, NULL, 3, &Task_Chassis_down_handle);
    xTaskCreate((TaskFunction_t)Task_Gimbal, "Task_Gimbal", 256, NULL, 4, &Task_Gimbal_handle);
    xTaskCreate((TaskFunction_t)Task_Shoot, "Task_Shoot", 256, NULL, 5, &Task_Shoot_handle);
    xTaskCreate((TaskFunction_t)Task_IMU_Rx, "IMU_Rx", 128, NULL, 6, &Task_IMU_handle);
    xTaskCreate((TaskFunction_t)Task_Remote_Rx, "Remote_Rx", 128, NULL, 6, &Task_Remote_handle);
    xTaskCreate((TaskFunction_t)Task_Protect, "Task_Protect", 128, NULL, 7, &Task_Protect_handle);
    
    taskEXIT_CRITICAL(); // 退出临界区
    
    vTaskDelete(NULL);   // 删除开始空闲任务

}
