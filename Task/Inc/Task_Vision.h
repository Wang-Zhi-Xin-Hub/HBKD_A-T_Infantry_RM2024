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

#define AM02_ARMOR_X  14
#define AM02_ARMOR_Y  12.5
#define AM12_ARMOR_X  23.5
#define AM12_ARMOR_Y  12.7

/* 目标位姿 */
typedef struct{
        float X;
        float Y;
        float Z;
        float HorizontalDistance;
}Pose_t;

/* 储存自瞄信息 */
typedef struct{
    int8_t Rx_flag;                   //!< @brief 接收状态
    
    Pose_t Predicted_PoseN;           //!< @brief 惯性系下的中心位姿
    Pose_t Predicted_Armor_Pose[4];   //!< @brief 目标装甲板位姿
    
    float Predicted_Center_time;      //!< @brief 预测时间
    float Predicted_Armor_time;
    uint16_t  Fixed_Center_time;               //静态预测时间(拨弹盘转动，通信延迟等)
    uint16_t  Fixed_Armor_time;                  //静态预测时间(拨弹盘转动，通信延迟等)
    
    float P_thre;                      //!< @brief 打弹阈值
    float Y_thre; 
    float K;                          //!< @brief 空气阻力
    float PitchOffset;                 //!< @brief 弹道补偿
    float Predicted_Yaw;               //!< @brief 预测的Y轴角度
    float Predicted_Pitch;             //!< @brief 预测的P轴角度   
    uint8_t aim_runing;                //!< @brief 自瞄启动
    int64_t TimeStamp_setoff;          //!< @brief 补偿单片机时间戳
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
    }Quaternions;            //!< @brief 四元数
    uint16_t Time_Gap;
    struct{
    float X;
    float Y;
    float Z;
    }Tar_Pose;              //!< @brief 预测击打点位姿
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
        uint8_t armor_number;            //!< @brief 装甲板的数量
        uint8_t armor_id;                //!< @brief 目标车辆ID
    }pose;                               //!< @brief 目标车辆位姿
    uint8_t delay;                       //!< @brief 视觉程序延迟
    uint8_t tracker_status;              //!< @brief 相机追踪状态
} Aim_Rx_info;
#pragma pack()
extern Aim_Rx_info Aim_Rx_infopack;

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
* @brief 坐标点到原点的水平距离
*/
float DistanceHorizontal(Pose_t pose);

/**
* @brief 坐标点到原点的距离
*/
float DistanceToOrigin(Pose_t pose);

/**
* @brief 空气阻力模型
*/
float monoAirResistance_Model(float horizontal, float bullet_speed, float angle_pitch);

/**
* @brief 获得飞行时间
*/
float Get_bullet_fly_time( float horizontal, float bullet_speed, float angle_pitch);

/**
* @brief 完全空气阻力模型
*/
double ballisticSolver(float horizontal, float vertical, double bullet_speed, double k);

/**
* @brief Pitch轴弹道补偿
*/
float Get_Pitch_Angle_Compensation(float horizontal, float vertical, float bullet_speed);

/**
* @brief 根据最优决策得出被击打装甲板 自动解算弹道
*/
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z);
#endif
