/*!
 * @file    Task_Init.h
 * @date    2024-1-1
 * @brief   初始化任务头文件
 */

/*! @mainpage
 * <table>
 * <tr><th>Project  <td><td>步兵机器人上板F407VGT6代码
 * <tr><th>Author   <td><td>RM王志鑫
 * <tr><th>Date     <td><td>2024-1-1
 * </table>
 *
 * @section 步兵机器人上板F407VGT6代码：
 * ##不同机器人需修改ROBOT_ID(Variate.h中)
 *
 * CAN1:上下板通信(上板发送ID：0X110（底盘速度）0X120（机构状态）,下板发送ID：0X101（上板所需裁判系统信息）)(线过滑环，双线备用))<br>
 *      拨弹盘2006电机（C610电调ID:1）(发送ID：0X200[0] 接收ID：0X201)<br>
 *      两个摩擦轮3508电机(C620电调ID：2[LEFT] 3[RIGHT]) （发送ID：0X200[1][2] 接收ID：0X202 0X203）
 *
 * CAN2:两个云台6020电机(ID：1[PITCH] 2[YAW])    (发送ID：0X1FF[0][1] 接收ID：0X205 0X206）（YAW轴电机CAN线过滑环连上板CAN2)
 *
 * USART1:NDJ6遥控器<br>
 * USART2:WT931陀螺仪
 *
 * 虚拟串口：与PC通信
 *
 * 看门狗： 遥控器 + 陀螺仪 + 2个云台电机 + 3个发射机构电机 + 上下板通信（5组看门狗对应5个LED灯（有规律能快速找到问题），灯亮则死）(灯可能容易坏上电时一瞬间应该都是亮的)
 *
 * ###Task代码运行逻辑：
 * 先在  StartDefaultTask(Freertos.c中的开始任务)中调用Task_Init()初始化外设，并删除开始任务<br>
 * 之后与   Task_Chassis_down.c (底盘控制（下板通信）任务)<br>
 *          Task_Gimbal.c (云台控制任务)<br>
 *          Task_Shoot.c (发射控制任务)<br>
 *          Task_Vision.c (视觉（自瞄）任务)<br>
 *          Task_Remote.c (遥控器数据处理)<br>
 *          Task_Protect.c (断控保护看门狗任务)同时运行.<br>
 * 模块主任务有任务调度，主任务会调用该模块中其他函数
 *
 *  ###User目录
 *  Variate.c中为全局变量定义、PID结构体定义，状态等<br>
 *  Variate.h中有全局宏定义、声明外部变量、枚举状态量结构体定义等<br>
 *  Function.c中为全局调用功能函数<br>
 *  Function.h中为宏定义函数、函数声明<br>
 *  Callback_Function.c为can回调函数，看门狗回调函数<br>
 *  IMU.c为陀螺仪模块<br>
 *  IMU.h为陀螺仪头文件
 *
 * <table>
 * <tr><th>初始化任务  <td>@see Task_Init.h @see Task_Init.c
 * <tr><th>底盘控制（下板通信）任务  <td>@see Task_Chassis_down.h @see Task_Chassis_down.c
 * <tr><th>云台控制任务  <td>@see Task_Gimbal.h @see Task_Gimbal.c
 * <tr><th>发射控制任务  <td>@see Task_Shoot.h @see Task_Shoot.c
 * <tr><th>视觉(自瞄)任务  <td>@see Task_Vision.h @see Task_Vision.c
 * <tr><th>陀螺仪解包任务  <td>@see Task_Remote.h @see Task_Remote.c
 * <tr><th>断控保护看门狗任务  <td>@see Task_Protect.h @see Task_Protect.c
 * </table>
 */

#ifndef __TASK_INIT_H
#define __TASK_INIT_H

#include "Variate.h"
#include "Function.h"

TaskHandle_t Task_Chassis_down_handle;
TaskHandle_t Task_Gimbal_handle;
TaskHandle_t Task_Shoot_handle;
TaskHandle_t Task_Communication_Vision_handle;
TaskHandle_t Task_IMU_handle;
TaskHandle_t Task_Remote_handle;
TaskHandle_t Task_Protect_handle;

/**
 * @brief 底盘控制（下板通信）任务
 */
void Task_Chassis_down(void *pvParameters);

/**
 * @brief 云台控制任务
 */
void Task_Gimbal(void *pvParameters);

/**
 * @brief 发射控制任务
 */
void Task_Shoot(void *pvParameters);

/**
 * @brief IMU解包任务
 */
void Task_IMU_Rx(void *pvParameters);

/**
 * @brief 遥控器解包任务
 */
void Task_Remote_Rx(void *pvParameters);

/**
 * @brief 视觉通信任务
 */
void Task_Communication_Vision(void *pvParameters);

/**
 * @brief 断控保护任务
 */
void Task_Protect(void *pvParameters);

#endif
