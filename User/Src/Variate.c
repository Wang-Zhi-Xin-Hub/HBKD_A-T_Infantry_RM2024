/*!
* @file     Variate.c
* @date     2024-1-1
* @brief    定义全局变量及PID结构体
*/
#include "Variate.h"

/* PID结构体定义(在此修改PID值) */

#if ROBOT_ID == 3
    /* 云台 */
PID Gimbal_Speed_PID[2][3] = {{{.Kp = 3.3f, .Ki = 0, .Kd = 1.5f, .limit = 5000},       //Pitch轴6020电机 0           初始归中 0
                               {.Kp = 4, .Ki = 0, .Kd = 5, .limit = 5000},             //  速度环                    机械 1
                               {.Kp = 30, .Ki = 0, .Kd = 3, .limit = 5000}},           //                            陀螺仪 2

                              {{.Kp = 4, .Ki = 0, .Kd = 5, .limit = 5000},            //Yaw轴6020电机 1         初始归中 0
                               {.Kp = 4.5f, .Ki = 0, .Kd = 3, .limit = 5000},         //                        机械 1
                               {.Kp = 30, .Ki = 0, .Kd = 3, .limit = 5000}}};         //                        陀螺仪 2

PID_Smis Gimbal_Place_PIDS[2][3] = {{{.Kp = 7, .Ki = 0, .Kd = 2.5, .limit = 5000},         //Pitch轴6020电机 0      初始 0
                                    {.Kp = 20, .Ki = 0, .Kd = 6, .limit = 5000},           //    位置环             机械 1
                                    {.Kp = 60, .Ki = 0, .Kd = -6, .limit = 5000}},          //                       陀螺仪 2

                                    {{.Kp = 5, .Ki = 0, .Kd = 3, .limit = 5000},           //Yaw轴6020电机 1       初始 0
                                    {.Kp = 15, .Ki = 0, .Kd = 3, .limit = 5000},           //                      机械 1
                                    {.Kp = 70, .Ki = 0, .Kd = -7, .limit = 5000}}};         //                      陀螺仪 2
    /* 发射机构 */
PID Shoot_Speed_PID[2] = {{.Kp = 5, .Ki = 0, .Kd = 0, .limit = 5000},            //摩擦轮左
                          {.Kp = 5, .Ki = 0, .Kd = 0, .limit = 5000}};           //摩擦轮右

PID_Smis Pluck_Place_PIDS = {.Kp = 3, .Ki = 0, .Kd = -1, .limit = 5000};         //拨弹盘单发位置环
PID Pluck_Speed_PID = {.Kp = 4, .Ki = 0, .Kd = 3, .limit = 5000};                //拨弹盘单发速度环
PID Pluck_Continue_PID = {.Kp = 15, .Ki = 0, .Kd = 0, .limit = 5000};            //拨弹盘连发模式
    /* 底盘跟随 */
PID_Smis Chassis_Speed_PIDS = {.Kp = 3.0f, .Ki = 0, .Kd = 6, .limit = 5000};     //底盘跟随   位置环
PID Chassis_Speed_PID = {.Kp =3.0f, .Ki = 0, .Kd = 0, .limit = 5000};           //           速度环

/* 卡尔曼初始化（用于自瞄滤波预测）*/
kalman_filterII_t Y_Kalman={
	.P_data = {2,0,0,2},              //协方差矩阵
    .A_data = {1, 0.001, 0, 1},       //预测矩阵（采样时间）
    .H_data = {1, 0, 0, 1}, 	      //传感器测量数据矩阵																		
    .Q_data = {1, 0, 0, 1},           //外部的不确定性（过程噪声协方差）     
    .R_data = {0.5,0,0,0.05},         //传感器测量方差（采集数据方差）
};
kalman_filterII_t P_Kalman={
	.P_data = {2,0,0,2},
    .A_data = {1, 0.001, 0, 1},
    .H_data = {1, 0, 0, 1},
    .Q_data = {1,0 , 0, 0.1},
    .R_data = {0.5,0,0,0.05},
};

#elif ROBOT_ID == 4

#endif

/* 上下板通信 */
Communication_Speed_t Communication_Speed_Tx = {0};
Communication_Action_t Communication_Action_Tx;
Referee_data_t  Referee_data_Rx = {0};

/* 视觉自瞄通信 */
Aim_Rx_info Aim_Rx_infopack;
Aim_Rx_t Aim_Rx = { .Yaw_Angle_Offset = -2, .Pitch_Angle_Offset = -10, .Rx_Flag = -1};
Aim_Tx_t Aim_Tx;

/* 雷达导航通信 */
Radar_Chassis_Speed_Ref_t Radar_Chassis_Speed;

/* 云台电机期望值 */
PTZAngle_Ref_t Gyro_Ref;
PTZAngle_Ref_t Mech_Ref;
PTZAngle_Ref_t Aim_Ref;
PTZAngle_Ref_t Gimbal_Ramp_Angle;
float Gimbal_increase[2][2]={0};

/* 串口缓存区 */
uint8_t Remote_flag = 0;
uint8_t IMU_flag    = 0;
uint8_t Usart1_Remote_Dma[2][Remote_Usart1_Len];
uint8_t Usart2_IMU_Dma[2][IMU_Usart2_Len];

/* 遥控器处理值 */
float Key_ch[4]   = {0};
float Mouse_ch[3] = {0};
InputMode_e RemoteMode;

/* CAN发送电机数据缓存区 */
int16_t Can1Send_Shoot[4] ={0};
int16_t Can2Send_Gimbal[4]={0};

/* 看门狗结构体定义 */
WatchDog_TypeDef Remote_Dog, Down_Dog, Gimbal_Dog[2], Shoot_Dog[2], Pluck_Dog, IMU_Dog, PC_Dog;

/* 电机数据结构体 */
GM6020_TypeDef Gimbal_Motor[2];
RM3508_TypeDef Shoot_Motor[2];
M2006_TypeDef  Pluck_Motor;

/* 设备状态 */
eSystemState systemState = SYSTEM_STARTING;
eDeviceState Remote_State, IMU_State, Gimbal_State[2], Shoot_State[2], Pluck_State, Down_State, PC_State;

/* 机构运行状态 */
eChassisAction ChassisAction = CHASSIS_FOLLOW;
eShootAction ShootAction     = SHOOT_STOP;
eAimAction AimAction         = AIM_STOP;
eCtrlMode CtrlMode           = GYRO_MODE;
eMidMode MidMode             = FRONT;
