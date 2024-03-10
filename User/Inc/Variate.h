/*!
 * @file     Variate.h
 * @date     2024-1-1
 * @brief    全局宏定义，引用外部变量，定义结构体头文件
 */
#ifndef __VARIATE_H
#define __VARIATE_H
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "motor.h"
#include "PID.h"
#include "remote.h"
#include "WatchDog.h"
#include "RMQueue.h"
#include "Chassis.h"
#include "CANDrive.h"
#include "IMU.h"
#include "Referee_offcial.h"
#include "math.h"
#include "stm32f4xx.h"
#include "VCOMCOMM.h"
#include "kalmanII.h"
#include "usbd_def.h"
#include "arm_math.h"

/** @brief  3/4号步兵  */
#define ROBOT_ID 3

/** @brief 不同ROBOT_ID不同参数 */
#if ROBOT_ID == 3
#define Yaw_Mid_Front 7880   //!< @brief Yaw轴电机机器人前方中值
#define Pitch_Mid 675        //!< @brief Pitch电机云台水平值
#define P_UP_limit 1200	     //!< @brief Pitch轴电机上限位
#define P_DOWN_limit 300     //!< @brief Pitch轴电机下限位
#define IMU_UP_limit 25      //!< @brief IMU上限位
#define IMU_DOWN_limit -20   //!< @brief IMU下限位
#define SHOOT_SPEED  4000    //!< @brief 摩擦轮电机速度环PID的期望值
#define PLUCK_SPEED 1300     //!< @brief 拨弹盘电机连发时速度环PID的期望值 //新拨弹盘8000
#define PLUCK_MOTOR_ONE 1340 //!< @brief 一发弹丸拨弹盘电机转过的机械角度

#elif ROBOT_ID == 4
#define Yaw_Mid_Front 7880   //!< @brief Yaw轴电机机器人前方中值
#define Pitch_Mid 675        //!< @brief Pitch电机云台水平值
#define P_UP_limit 1200	     //!< @brief Pitch轴电机上限位
#define P_DOWN_limit 300     //!< @brief Pitch轴电机下限位
#define SHOOT_SPEED  5800    //!< @brief 摩擦轮电机速度环PID的期望值
#define PLUCK_SPEED -1380     //!< @brief 拨弹盘电机连发时速度环PID的期望值
#define PLUCK_MOTOR_ONE -1340 //!< @brief 一发弹丸拨弹盘电机转过的机械角度
#endif

/* 便于单独测试各机构，如果启动的机构电机不全在线，需注释对应任务里的电机在线判断 */
/** @brief    是否启动底盘电机驱动 */
#define CHASSIS_RUN 1
/** @brief    是否启动云台电机驱动 */
#define GIMBAL_RUN  1
/** @brief    是否启动发射机构电机驱动 */
#define SHOOT_RUN   1

/** @brief 根据机器人正前方Yaw轴机械角度算出机器人剩下三个方向Yaw轴机械角度 */
#if (Yaw_Mid_Front + 2048) > 8191
#define Yaw_Mid_Right Yaw_Mid_Front - 6143
#else
#define Yaw_Mid_Right Yaw_Mid_Front + 2048
#endif

#if (Yaw_Mid_Right + 2048) > 8191
#define Yaw_Mid_Back Yaw_Mid_Right - 6143
#else
#define Yaw_Mid_Back Yaw_Mid_Right + 2048
#endif

#if (Yaw_Mid_Back + 2048) > 8191
#define Yaw_Mid_Left Yaw_Mid_Back - 6143
#else
#define Yaw_Mid_Left Yaw_Mid_Back + 2048
#endif

/** @brief 串口1遥控器数据长度 */
#define Remote_Usart1_Len   RC_FRAME_LENGTH + 1
/** @brief 串口2陀螺仪数据长度（由陀螺仪开启功能决定） */
#define IMU_Usart2_Len      IMU_LEN + 1
/** @brief 储存发送给视觉的角度的数组长度   */
#define Vision_Reserve_Angle_Len 25

/** @brief 电机数据代号 */
#define PITCH 0
#define YAW   1
#define LEFT   0
#define RIGHT  1

/** @brief 模式代号 */
#define Change_RC  0
#define Change_KEY 1
#define Init 0
#define Mech 1
#define Gyro 2

/** @brief 电机参考速度 */
#define STD_Speed 2380 // 标准速度1m/s
#define STD_Omega 6142 // 标准速度1rpm/s

// 前后速度
#define CHASSIS_Speed_H_X (STD_Speed * 2.5f) // 高速2.5m/s
#define CHASSIS_Speed_L_X (STD_Speed * 1.5f) // 平时速度1.5m/s

// 平移速度
#define CHASSIS_Speed_H_Y (STD_Speed * 2.5f) // 高速2.5m/s
#define CHASSIS_Speed_L_Y (STD_Speed * 1.5f) // 平时速度1.5m/s

// 旋转速度
#define CHASSIS_Speed_R (STD_Omega / 4) // 转向速度 0.25rpm/s

#define STD_Angle 0.72f			   // 角度制1rpm/s
#define STD_MAngle 16.384f		   // 机械角度制1rpm/s
#define A_Y (STD_Angle * 0.25f)	   // 角度制云台yaw速度参数0.25rpm/s
#define A_Y_1 (STD_Angle * 0.75f)  // 角度制云台yaw速度参数0.75rpm/s
#define A_Y_10 (STD_Angle * 0.10f) // 角度制云台yaw速度参数0.10rpm/s
#define A_Y_H (STD_Angle * 1.0f)
#define A_P (STD_Angle * 0.18f)	 // 角度制云台picth速度参数0.25rpm/s
#define M_Y (STD_MAngle * 0.25f) // 机械角度制云台yaw速度参数0.25rpm/s
#define M_P (STD_MAngle * 0.25f) // 机械角度制云台pitch速度参数0.25rpm/s

/** @brief 设备状态 */
typedef enum
{
	Device_Offline = 0,     //!< @brief 设备离线
	Device_Online  = 1,	    //!< @brief 设备在线
	Device_Error   = 2	    //!< @brief 设备错误
} eDeviceState;
extern eDeviceState Remote_State, IMU_State, Gimbal_State[2], Shoot_State[2], Pluck_State, Down_State, PC_State;

/** @brief  系统状态 */
typedef enum
{
	SYSTEM_STARTING = 0,     //!< @brief 正在启动
	SYSTEM_RUNNING  = 1,	 //!< @brief 正在运行
} eSystemState;
extern eSystemState systemState;

/** @brief  云台模式 */
typedef enum
{
	MECH_MODE = 0,       //!< @brief  机械,云台跟随底盘
	GYRO_MODE = 1,       //!< @brief  陀螺仪,底盘跟随云台
} eCtrlMode;
extern eCtrlMode CtrlMode;

/** @brief  云台归中位置 */
typedef enum
{
	FRONT = 0,       //!< @brief   前方
	BACK  = 1,       //!< @brief   后方
} eMidMode;
extern eMidMode MidMode;
/** @brief  自瞄状态 */
typedef enum
{
	AIM_STOP = 0,     //!< @brief   关闭自瞄
	AIM_AID  = 1,     //!< @brief   辅瞄不自动射击
	AIM_AUTO = 2,     //!< @brief   自瞄+自动射击
} eAimAction;
extern eAimAction AimAction;

/** @brief  射击模式 */
typedef enum
{
	SHOOT_STOP = 0,	        //!< @brief  停止发射（摩擦轮、拨弹盘停止）
	SHOOT_READY = 1,        //!< @brief  准备发射（摩擦轮启动）
	SHOOT_NORMAL = 2,       //!< @brief  单发(拨弹盘转动一个弹丸位置)
	SHOOT_RUNNING = 3,      //!< @brief  速射
    SHOOT_STUCKING = 4,     //!< @BRIEF  卡弹退弹中
} eShootAction;
extern eShootAction ShootAction;

/** @brief  底盘模式 */
typedef enum
{
	CHASSIS_FOLLOW = 0, //!< @brief   底盘跟随模式
	CHASSIS_SPIN = 1,	//!< @brief   小陀螺模式
	CHASSIS_NORMAL = 2,	//!< @brief   普通底盘（调试用）
    CHASSIS_RADAR = 3   //!< @brief   雷达导航模式
} eChassisAction;
extern eChassisAction ChassisAction;

enum{
    X = 0,
    Y = 1,
    Z = 2
};
/** @brief  下板发给上板(裁判系统） */
typedef struct
{
	int8_t robot_level;			   //!< @brief 机器人等级
	float shoot_speed_last;		   //!< @brief 上一发弹丸速度
	uint16_t shooter_cooling_heat; //!< @brief 实时枪口热量
} Referee_data_t;
extern Referee_data_t Referee_data_Rx;

/** @brief  上板发给下板 (底盘速度) */
typedef struct
{
	ChassisSpeed_Ref_t Chassis_Speed; //!< @brief 底盘期望速度
	uint8_t Shift_flag;				  //!< @brief Shift加速（也可作为其他状态标志位）
} Communication_Speed_t;
extern Communication_Speed_t Communication_Speed_Tx;

/** @brief  上板发给下板(状态UI） */
typedef struct
{
	eChassisAction ChassisAction_Tx; //!< @brief 底盘当前模式
	eShootAction ShootAction_Tx;	 //!< @brief 发射机构当前模式
	eAimAction AimAction_Tx;		 //!< @brief 自瞄当前模式
    eCtrlMode CtrlMode_Tx;			 //!< @brief 云台当前模式
	eMidMode MidMode_Tx;			 //!< @brief 当前归中位置
} Communication_Action_t;
extern Communication_Action_t Communication_Action_Tx;

/** @brief  视觉自瞄通信接收结构体 */
typedef struct
{
    int8_t Rx_Flag;                     //!< @brief 接收标志位（确定当前接收状态）
    float Predicted_PoseB[3];           //!< @brief 云台系下的位姿
    float Predicted_PoseN[3];           //!< @brief 惯性系下的位姿
	float Predicted_Yaw;                //!< @brief 预测的Y轴角度
	float Predicted_Pitch;              //!< @brief 预测的P轴角度
	float Predicted_Distance;           //!< @brief 预测的距离
    
    uint8_t Shoot_Flag;                 //!< @brief 射击标志位
  	uint8_t Tracker_Status;             //!< @brief 视觉发来的相机此时的跟踪状态 0丢失 1决策中 2跟踪中 3短暂丢失
    
	float Yaw_Angle_Offset;             //!< @brief IMU与枪管的Yay轴角度差
	float Pitch_Angle_Offset;           //!< @brief IMU与枪管的Pitch轴角度差
    
    uint8_t aim_start;                  //!< @brief 达到自瞄条件(可以自瞄)

    uint64_t StandardTimeStamp;         //!< @brief 时间戳与视觉对齐
    int64_t TimeStamp_setoff;           //!< @brief 补偿单片机时间戳
}Aim_Rx_t;
extern Aim_Rx_t Aim_Rx;

/** @brief  发送给视觉的结构体 */
typedef struct
{
    uint64_t TimeStamp;      //!< @brief 时间戳
    struct{
        float W;       
        float X;
        float Y;
        float Z;
    }Quaternions;            //!< @brief 四元数（视觉坐标转换）
} Aim_Tx_t;
extern Aim_Tx_t Aim_Tx;

/** @brief  接收视觉数据的结构体（内存对齐） */
#pragma pack(1)
typedef struct
{
    struct{
        float X;                         
        float Y;                           
        float Z;                          
        float Vx;
        float Vy;
        float Vz;
        float theta;
        float omega;
        float r;
    }pose;                               //!< @brief 目标位姿
    uint8_t delay;                       //!< @brief 视觉程序延迟
    uint8_t tracker_status;              //!< @brief 相机追踪状态
} Aim_Rx_info;
#pragma pack()
extern Aim_Rx_info Aim_Rx_infopack;

/* 雷达 */
typedef struct
{
    double forward_back_ref;  //!< @brief 前进方向速度
    double left_right_ref;    //!< @brief 左右方向速度
    double rotate_ref;        //!< @brief 旋转速度
}Radar_Chassis_Speed_Ref_t;
extern Radar_Chassis_Speed_Ref_t Radar_Chassis_Speed;

/** @brief 声明外部变量，引入其他任务头文件中 */
/* 遥控器 */
extern float Key_ch[4];
extern float Mouse_ch[3];
extern InputMode_e RemoteMode;
/* 电机 */
extern GM6020_TypeDef Gimbal_Motor[2];
extern RM3508_TypeDef Shoot_Motor[2];
extern M2006_TypeDef  Pluck_Motor;
/* PID */
extern PID Gimbal_Speed_PID[2][3], Shoot_Speed_PID[2], Pluck_Speed_PID, Pluck_Continue_PID, Chassis_Speed_PID;
extern PID_Smis Gimbal_Place_PIDS[2][3], Pluck_Place_PIDS, Chassis_Speed_PIDS;
/* 云台期望 */
extern PTZAngle_Ref_t Gyro_Ref, Mech_Ref, Aim_Ref, Gimbal_Ramp_Angle;
extern float Gimbal_increase[2][2];
/* 串口 */
extern uint8_t Remote_flag,IMU_flag;
extern uint8_t Usart1_Remote_Dma[2][Remote_Usart1_Len], Usart2_IMU_Dma[2][IMU_Usart2_Len];
/* Can */
extern int16_t Can1Send_Shoot[4], Can2Send_Gimbal[4];
/* 看门狗 */
extern WatchDog_TypeDef Remote_Dog, IMU_Dog, Gimbal_Dog[2], Shoot_Dog[2], Pluck_Dog, Down_Dog, PC_Dog;
/* 视觉 */
extern int8_t fun_code_rx;
extern int16_t id_rx;
extern kalman_filterII_t Y_Kalman, P_Kalman;
extern USBD_HandleTypeDef hUsbDeviceFS;
/* 任务句柄 */
extern TaskHandle_t  Task_Chassis_down_handle;
extern TaskHandle_t  Task_Gimbal_handle;
extern TaskHandle_t  Task_Shoot_handle;
extern TaskHandle_t  Task_Protect_handle;
extern TaskHandle_t  Task_IMU_handle;
extern TaskHandle_t  Task_Remote_handle;

#endif
