/*!
* @file     Variate.c
* @brief    定义全局变量及PID结构体
*/
#include "Variate.h"

/* PID结构体定义(在此修改PID值) */

#if ROBOT_ID == 3
    /* 云台 */
PID Gimbal_Speed_PID[GIMBAL_SUM][GIMBAL_MODE] = {{{.Kp = 3.3f, .Ki = 0, .Kd = 1.5f, .limit = 5000},      //Pitch轴6020电机 0           初始归中 0
                                            {.Kp = 12, .Ki = 0, .Kd = 2.5, .limit = 5000},             //  速度环                    机械 1
                                            {.Kp = 30, .Ki = 0, .Kd = 0, .limit = 5000}},             //                            陀螺仪 2

                                            {{.Kp = 4.5, .Ki = 0, .Kd = 3, .limit = 5000 },            //Yaw轴6020电机 1         初始归中 0
                                            {.Kp = 4.5f, .Ki = 0, .Kd = 3, .limit = 5000},             //                        机械 1
                                            {.Kp = 30, .Ki = 0, .Kd = 0, .limit = 5000}}};             //                        陀螺仪 2

PID_Smis Gimbal_Place_PIDS[GIMBAL_SUM][GIMBAL_MODE] = {{{.Kp = 7, .Ki = 0, .Kd = -2.5, .limit = 5000},         //Pitch轴6020电机 0      初始 0
                                                {.Kp = 25, .Ki = 0.1, .Kd = -30, .limit = 5000},              //    位置环             机械 1
                                                {.Kp = 75, .Ki = 0.10, .Kd = -6, .limit = 6000,.error_thre =0 ,.DeadBand = 0.00}},         //    陀螺仪 2

                                                {{.Kp = 5, .Ki = 0, .Kd = -18, .limit = 5000},           //Yaw轴6020电机 1       初始 0
                                                {.Kp = 15, .Ki = 0, .Kd = -3, .limit = 5000},              //                      机械 1
                                                {.Kp = 85, .Ki = 0, .Kd = -6.5, .limit = 5000}}};          //                      陀螺仪 2
    /* 发射机构 */
PID Shoot_Speed_PID[FRIC_SUM] = {{.Kp = 5, .Ki = 0, .Kd = 0, .limit = 5000},            //摩擦轮左
                          {.Kp = 5, .Ki = 0, .Kd = 0, .limit = 5000}};           //摩擦轮右

PID_Smis Pluck_Place_PIDS = {.Kp = 3, .Ki = 0, .Kd = -1, .limit = 5000};         //拨弹盘单发位置环
PID Pluck_Speed_PID = {.Kp = 4, .Ki = 0, .Kd = 3, .limit = 5000};                //拨弹盘单发速度环
PID Pluck_Continue_PID = {.Kp = 15, .Ki = 0, .Kd = 0, .limit = 5000};            //拨弹盘连发模式

   /* 底盘跟随 */
PID_Smis Chassis_Speed_PIDS = {.Kp = 5, .Ki = 0, .Kd = 0, .limit = 5000};     //底盘跟随   位置环
PID Chassis_Speed_PID = {.Kp =2, .Ki = 0, .Kd = 1, .limit = 5000};            //           速度环
FeedForward_Typedef Chassis_FF = {.K1 = 2000, .OutMax = RM3508_LIMIT};

#elif ROBOT_ID == 4

#endif

/* 上下板通信 */
Communication_Speed_t Communication_Speed_Tx = {0};
Communication_Action_t Communication_Action_Tx;
Referee_data_t  Referee_data_Rx = {0};

/* 雷达导航通信 */
Radar_Chassis_Speed_Ref_t Radar_Chassis_Speed;

/* 云台电机期望值 */
PTZAngle_Ref_t Gyro_Ref;
PTZAngle_Ref_t Mech_Ref;
PTZAngle_Ref_t Aim_Ref;
float Gimbal_increase[GIMBAL_SUM][RC_Mode]={0};

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
WatchDog_TypeDef Remote_Dog, Down_Dog, Gimbal_Dog[GIMBAL_SUM], Shoot_Dog[FRIC_SUM], Pluck_Dog, IMU_Dog, PC_Dog;

/* 电机数据结构体 */
GM6020_TypeDef Gimbal_Motor[GIMBAL_SUM];
RM3508_TypeDef Shoot_Motor[FRIC_SUM];
M2006_TypeDef  Pluck_Motor;

/* 设备状态 */
eSystemState systemState = SYSTEM_STARTING;
eDeviceState Remote_State, IMU_State, Gimbal_State[GIMBAL_SUM], Shoot_State[FRIC_SUM], Pluck_State, Down_State, PC_State;

/* 机构运行状态 */
eChassisAction ChassisAction = CHASSIS_FOLLOW;
eShootAction ShootAction     = SHOOT_STOP;
eAimAction AimAction         = AIM_STOP;
eCtrlMode CtrlMode           = GYRO_MODE;
eMidMode MidMode             = FRONT;
