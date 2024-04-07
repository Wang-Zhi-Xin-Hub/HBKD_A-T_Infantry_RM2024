/*!
 * @file     Task_Vision.h
 * @date     2024-1-1
 * @auther   Wzx
 * @brief    视觉通信（自瞄）任务头文件
 */
#ifndef __TASK_VISION_H
#define __TASK_VISION_H

#include "Variate.h"
#include "Function.h"

/* 目标位姿 */
typedef struct{
        float X;
        float Y;
        float Z;
        float Distance;
}Pose_t;

/* 储存自瞄信息 */
typedef struct
{
    int8_t Rx_flag;                     //!< @brief 接收状态
    
    Pose_t Predicted_PoseB;           //!< @brief 云台系下的位姿
    Pose_t Predicted_PoseN;           //!< @brief 惯性系下的位姿
    Pose_t Predicted_Armor_Pose[4];   //!< @brief 装甲板位姿
    
    float Predicted_Yaw;                //!< @brief 预测的Y轴角度
    float Predicted_Pitch;              //!< @brief 预测的P轴角度

	float Yaw_Angle_Offset;             //!< @brief IMU与枪管的Yay轴角度差
	float Pitch_Angle_Offset;           //!< @brief IMU与枪管的Pitch轴角度差
    
    uint8_t aim_runing;                  //!< @brief 自瞄启动
    
    int64_t TimeStamp_setoff;           //!< @brief 补偿单片机时间戳
}Aim_Rx_t;
extern Aim_Rx_t Aim_Rx;

/* 发送给视觉的结构体 */
typedef struct
{
    uint64_t TimeStamp;      //!< @brief 对齐后的时间戳
    struct{
        float W;       
        float X;
        float Y;
        float Z;
    }Quaternions;            //!< @brief 四元数（视觉坐标转换）
    uint16_t Time_Gap;
    float X;
    float Y;
    float Z;
} Aim_Tx_t;
extern Aim_Tx_t Aim_Tx;

/* 接收视觉数据的结构体（内存对齐） */
#pragma pack(1)
typedef struct
{
    struct{
        float X;                        //!< @brief 全局坐标系下车辆中心点的坐标
        float Y;
        float Z;
        float Vx;                       //!< @brief 全局坐标系下车辆中心点的线速度
        float Vy;
        float Vz;
        float theta;                    //!< @brief 目标装甲板朝向角
        float omega;                    //!< @brief 目标装甲板朝向角的角速度
        float r1;                       //!< @brief 目标中心到前后装甲板的距离
        float r2;                       //!< @brief 目标中心到左右装甲板的距离
        float dz;                       //!< @brief 另一对装甲板的相对于被跟踪装甲板的高度差
        int8_t armor_number;            //!< @brief 装甲板的数量
    }pose;                               //!< @brief 目标车辆位姿
    uint8_t delay;                       //!< @brief 视觉程序延迟
    uint8_t tracker_status;              //!< @brief 相机追踪状态
} Aim_Rx_info;
#pragma pack()
extern Aim_Rx_info Aim_Rx_infopack;

/* 自瞄装甲板所属机器人ID */
enum ARMOR_ID{
    ARMOR_OUTPOST = 0,               //!< @brief 前哨战
    ARMOR_HERO = 1,                  //!< @brief 英雄
    ARMOR_ENGINEER = 2,              //!< @brief 工程 
    ARMOR_INFANTRY3 = 3,             //!< @brief 步兵3    
    ARMOR_INFANTRY4 = 4,             //!< @brief 步兵4    
    ARMOR_INFANTRY5 = 5,             //!< @brief 步兵5    
    ARMOR_GUARD = 6,                 //!< @brief 哨兵 
    ARMOR_BASE = 7                   //!< @brief 基地  
};

/* 自瞄装甲类型 */
enum ARMOR_TYPE{
    ARMOR_NUM_BALANCE = 2,          //!< @brief 平衡底盘（2块装甲板）
    ARMOR_NUM_OUTPOST = 3,          //!< @brief 前哨战（3块装甲板）
    ARMOR_NUM_NORMAL = 4            //!< @brief 普通底盘（4块装甲板）
};

/* 弹丸类型 */
typedef enum {
    BULLET_17mm = 17,                //!< @brief 17mm弹丸
    BULLET_42mm = 42                 //!< @brief 42mm弹丸
}BULLET_TYPE;

/* 弹道解算设置参数 */
typedef struct {
    float k;                     //!< @brief 弹道系数

    /* 自身参数 */
    BULLET_TYPE bullet_type;      //!< @brief 自身机器人类型 0-步兵 1-英雄
    float current_v;              //!< @brief 当前弹速
    float current_pitch;          //!< @brief 当前pitch
    float current_yaw;            //!< @brief 当前yaw

    /* 目标参数 */
    float xw;             //!< @brief ROS坐标系下的x
    float yw;             //!< @brief ROS坐标系下的y
    float zw;             //!< @brief ROS坐标系下的z
    float vxw;            //!< @brief ROS坐标系下的vx
    float vyw;            //!< @brief ROS坐标系下的vy
    float vzw;            //!< @brief ROS坐标系下的vz
    float tar_yaw;        //!< @brief 目标yaw
    float v_yaw;          //!< @brief 目标yaw速度
    float r1;             //!< @brief 目标中心到前后装甲板的距离
    float r2;             //!< @brief 目标中心到左右装甲板的距离
    float dz;             //!< @brief 另一对装甲板的相对于被跟踪装甲板的高度差
    int bias_time;        //!< @brief 偏置时间
    float s_bias;         //!< @brief 枪口前推的距离
    float z_bias;         //!< @brief yaw轴电机到枪口水平面的垂直距离
    enum ARMOR_ID armor_id;         //!< @brief 装甲板所属机器人ID
    enum ARMOR_TYPE armor_type;     //!< @brief 装甲板类型
}Solve_Trajectory_Params_t;
extern Solve_Trajectory_Params_t st;
/* 用于存储目标装甲板的信息 */
struct tar_pose{
    float x;           //!< @brief 装甲板在世界坐标系下的x
    float y;           //!< @brief 装甲板在世界坐标系下的y
    float z;           //!< @brief 装甲板在世界坐标系下的z
    float yaw;         //!< @brief 装甲板坐标系相对于世界坐标系的yaw角
};

/**
* @brief 自瞄控制主程序
*/
void Aim_Control(void);

/**
* @brief 给视觉发送数据

*/
void Send_to_Vision(void);
/**
* @brief 坐标转换(N->B)
*/
void Coordinate_Transformation (RotationMatrix_t R, const Pose_t* PoseN, Pose_t* PoseB);
    
/**
* @brief 坐标点到原点的距离
*/
float DistanceToOrigin(Pose_t pose);

/**
* @brief 空气阻力模型
* @param s:m   云台距离装甲板的距离
* @param v:m/s 子弹飞行速度
* @param *t:s  子弹飞行时间,传入会直接修改成子弹飞行时间
* @param p_angle  当前云台Pitch轴角度，需校准IMU水平位置为0
* @return z:m  竖直方向需要补偿的的高度
*/
float monoAirResistance_Model(float horizontal, float bullet_speed, float angle_pitch);

/**
* @brief 获得飞行时间
*/
float Get_bullet_fly_time( float horizontal, float bullet_speed, float angle_pitch);

/**
* @brief 完全空气阻力模型
*/
float completeAirResistanceModel(float s, float v, float angle);

/**
* @brief Pitch轴弹道补偿
*/
float Get_Pitch_Angle_Compensation(float horizontal, float vertical, float bullet_speed);

/**
* @brief 根据最优决策得出被击打装甲板 自动解算弹道
*/
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z);
#endif
