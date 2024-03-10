/*!
 * @file     Task_Vision.c
 * @date     2024-3-10
 * @auther   Wzx
 * @brief    视觉通信（自瞄）任务
 */
#include "Task_Vision.h"

/* 陀螺仪接收解包/发送给视觉任务 */
void Task_IMU_Rx(void *pvParameters)
{
    static uint8_t Hz_Flag = 0;
    for(;;)
    {
        if(osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever))
        {
            if(IMU_flag)
                IMU_Receive(&IMU,Usart2_IMU_Dma[0]);
            else
                IMU_Receive(&IMU,Usart2_IMU_Dma[1]);
            Send_to_Vision();   //接收到IMU数据立即发送给视觉
        }
    }
}

/* 给视觉发送信息 */
void Send_to_Vision()
{
    static uint8_t Count = 0,Count_Max = 4; //控制发送频率，根据IMU频率更改，发送频率 = IMU频率 / Count_Max
    
    /* Get Vision TimeStamp*/
    if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && Aim_Rx.Rx_Flag == -1)
    {
        VCOMM_Transmit(0, 1, (uint8_t *)"Time", sizeof("Time"));
    }
    
    /* Send Messgae */
    if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && ++Count == Count_Max && Aim_Rx.Rx_Flag != -1)
    {   
        Aim_Tx.TimeStamp     = xTaskGetTickCount() +  Aim_Rx.TimeStamp_setoff;
        Aim_Tx.Quaternions.W = IMU.Quaternions.W;
        Aim_Tx.Quaternions.X = IMU.Quaternions.X;
        Aim_Tx.Quaternions.Y = IMU.Quaternions.Y;
        Aim_Tx.Quaternions.Z = IMU.Quaternions.Z;
        VCOMM_Transmit(1, 1, (uint8_t *)&Aim_Tx, sizeof(Aim_Tx));
        Count = 0;
    }
}

/* 自瞄控制 */
void Aim_Control()
{
    /* 自瞄系统正在测试中 */
    static uint16_t Shoot_time_Gap = 0;
    static uint16_t Predicted_time = 40 ,Predicted_Gap = 0,Bullet_fly_time = 0, Vision_need_time = 0;
    
      /* 自瞄模式 */
    if ( AimAction != AIM_STOP && PC_State == Device_Online)
    {
        /* 过程更新 */
        if( Aim_Rx.Rx_Flag )
        {
            Aim_Rx.Rx_Flag = 0;
            Vision_need_time = Aim_Rx_infopack.delay;
            Predicted_Gap = 0;
        }
        else/* 量测更新 */
            Predicted_Gap += 1;
        
        /* 预测时间 */
        Predicted_time = 100 + Vision_need_time + Bullet_fly_time;
        
        /* 角度更新(预测) */
        Aim_Rx.Predicted_PoseN[X] = (Aim_Rx_infopack.pose.X + Aim_Rx_infopack.pose.Vx * (Predicted_time + Predicted_Gap));
        Aim_Rx.Predicted_PoseN[Y] = (Aim_Rx_infopack.pose.Y + Aim_Rx_infopack.pose.Vy * (Predicted_time + Predicted_Gap));
        Aim_Rx.Predicted_PoseN[Z] = (Aim_Rx_infopack.pose.Z + Aim_Rx_infopack.pose.Vz * (Predicted_time + Predicted_Gap));
//        Aim_Rx.Predicted_PoseN[X] = (Aim_Rx_infopack.pose.X);
//        Aim_Rx.Predicted_PoseN[Y] = (Aim_Rx_infopack.pose.Y);
//        Aim_Rx.Predicted_PoseN[Z] = (Aim_Rx_infopack.pose.Z);
        
        /* 坐标转换(N—>B) */
        Coordinate_Transformation(IMU.RotationMatrix, Aim_Rx.Predicted_PoseN, Aim_Rx.Predicted_PoseB);
        
        /* 换算角度 */
        Aim_Rx.Predicted_Yaw      = atan2( Aim_Rx.Predicted_PoseB[Y], -Aim_Rx.Predicted_PoseB[X]) * 180 / PI + Aim_Rx.Yaw_Angle_Offset;
        Aim_Rx.Predicted_Pitch    = atan2( Aim_Rx.Predicted_PoseB[Z], -Aim_Rx.Predicted_PoseB[X]) * 180 / PI + Aim_Rx.Pitch_Angle_Offset;
        Aim_Rx.Predicted_Distance = DistanceToOrigin(Aim_Rx.Predicted_PoseB[X], Aim_Rx.Predicted_PoseB[Y], Aim_Rx.Predicted_PoseB[Z]);
        
        /* 弹道模型TODO */
        Bullet_fly_time = Aim_Rx.Predicted_Distance / 25 * 1000;
        
        /* 获得期望角度 */
        Aim_Ref.Yaw   = IMU.EulerAngler.ContinuousYaw + Aim_Rx.Predicted_Yaw;
        Aim_Ref.Pitch = IMU.EulerAngler.Pitch         - Aim_Rx.Predicted_Pitch ;
        
        /* 能否开启自瞄 */
        if( ( Aim_Rx.Tracker_Status == 2 ) || ( Aim_Rx.Tracker_Status == 3 ) )
        {
            /* 自瞄启动 */
            Aim_Rx.aim_start = 1;

//            /* 自瞄时遥控器不可控(记录角度) */
//            Gyro_Ref.Yaw   = IMU.EulerAngler.ContinuousYaw;
//            Gyro_Ref.Pitch = IMU.EulerAngler.Pitch;
        }
        else
            Aim_Rx.aim_start = 0;

        /* 自瞄启动自动打弹 */
        if( AimAction == AIM_AUTO && Aim_Rx.aim_start && ShootAction != SHOOT_STOP)
        {
            if( ABS (IMU.EulerAngler.ContinuousYaw - Aim_Ref.Yaw) <= 0.5f && ABS (IMU.EulerAngler.Pitch - Aim_Ref.Pitch) <= 8.5f )
                Aim_Rx.Shoot_Flag = 1;
            else
                Aim_Rx.Shoot_Flag = 0;
            
            if(Aim_Rx.Shoot_Flag && !Shoot_time_Gap)
            {
                Shoot_time_Gap = 500;
                if(ShootAction != SHOOT_STUCKING)
                ShootAction = SHOOT_NORMAL;
            }
            else
            {
              if(Shoot_time_Gap)
                  Shoot_time_Gap--;
              if(ShootAction != SHOOT_STUCKING)
                ShootAction = SHOOT_READY;
            }
        }
    }
    else
    {
        Aim_Rx.aim_start = 0;
    }
}

/* 坐标转换(N->B) */
void Coordinate_Transformation (RotationMatrix_t R, const float* PoseN, float* PoseB)
{
    RotationMatrix_t RT;
    
    // 创建矩阵
    arm_matrix_instance_f32 matR;
    arm_matrix_instance_f32 matRT;  
    arm_matrix_instance_f32 matP;
    arm_matrix_instance_f32 matResult;

    // 初始化矩阵
    arm_mat_init_f32(&matR, 3, 3, (float32_t *)R.r);
    arm_mat_init_f32(&matRT, 3, 3, (float32_t *)RT.r);    
    arm_mat_init_f32(&matP, 3, 1, (float32_t *)PoseN);
    arm_mat_init_f32(&matResult, 3, 1, (float32_t *)PoseB);
    
    //转置旋转矩阵
    arm_mat_trans_f32(&matR,&matRT);

    // 矩阵相乘
    arm_mat_mult_f32(&matRT, &matP, &matResult);
}

/* 坐标点到原点的距离 */
float DistanceToOrigin(float X, float Y, float Z){
    return sqrt(  X * X + Y * Y + Z * Z);
}

