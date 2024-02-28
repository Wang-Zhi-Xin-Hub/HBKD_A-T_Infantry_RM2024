/*!
 * @file     Task_Vision.c
 * @date     2024-1-1
 * @auther   王志鑫
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
            if(IMU_flag == 1)
                IMU_Receive(&IMU,Usart2_IMU_Dma[0]);
            else
            {
                IMU_Receive(&IMU,Usart2_IMU_Dma[1]);
                
                if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && Aim_Rx.Rx_ID == -1)
                {
                    /* Get Vision TimeStamp*/
                    VCOMM_Transmit(0, 1, (uint8_t *)"Time", sizeof("Time"));
                    /* Kalman_init */
                    kalmanII_Init(&Y_Kalman); 
                    kalmanII_Init(&P_Kalman); 
                }
                if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && Hz_Flag++ == 1 && Aim_Rx.Rx_ID != -1)
                {
                    /* 接收到4次IMU数据立即发送当前次给视觉（125Hz） */
                    Aim_Tx.TimeStamp     = xTaskGetTickCount() +  Aim_Rx.TimeStamp_setoff;
                    Aim_Tx.Quaternions.W = IMU.Quaternions.W;
                    Aim_Tx.Quaternions.X = IMU.Quaternions.X;
                    Aim_Tx.Quaternions.Y = IMU.Quaternions.Y;
                    Aim_Tx.Quaternions.Z = IMU.Quaternions.Z;
                    VCOMM_Transmit(1, 1, (uint8_t *)&Aim_Tx, sizeof(Aim_Tx));
                    /* 将发送给视觉时陀螺仪的角度缓存起来 */
                    Aim_Rx.Yaw_Send_Angle[Aim_Tx.Tx_ID]     = IMU.EulerAngler.ContinuousYaw;
                    Aim_Rx.Pitch_Send_Angle[Aim_Tx.Tx_ID++] = IMU.EulerAngler.Pitch;
                    /* 缓存长度由发送接收ID之间延迟差决定（环形缓冲区ID索引）*/
                    Aim_Tx.Tx_ID = Aim_Tx.Tx_ID % Vision_Reserve_Angle_Len;
                    Hz_Flag = 0;
                }
            }
        }
    }
}

/* 自瞄启动逻辑 */
void Aim_Control()
{
    /* 自瞄系统正在测试中 */
    static uint8_t Last_ID = 0;
    
      /* 自瞄模式 */
    if ( AimAction != AIM_STOP && PC_State == Device_Online)
    {
        /* 打弹逻辑 */
        if( AimAction == AIM_AUTO && Aim_Rx.aim_start && ShootAction != SHOOT_STOP)
        {
            if(Aim_Rx.Shoot_Flag)
                ShootAction = SHOOT_NORMAL;
            else
                ShootAction = SHOOT_READY;
        }
            /* Kalman Hz太低，后续需优化 */
        if( Last_ID != Aim_Rx.Rx_ID )
        {
            /* 相机跟踪中/短暂丢失 */
            if( ( Aim_Rx.Tracker_Status == 2 ) || ( Aim_Rx.Tracker_Status == 3 ) )
            {
                /* 自瞄角度 */
                Aim_Rx.Aim_Yaw_Now   = Aim_Rx.Yaw_Send_Angle[Aim_Rx.Rx_ID]   + Aim_Rx.Yaw_Angle;
                Aim_Rx.Aim_Pitch_Now = Aim_Rx.Pitch_Send_Angle[Aim_Rx.Rx_ID] + Aim_Rx.Pitch_Angle;
                
                /* 速度计算（°/s）（不准）*/
                if( Aim_Rx.Rx_Time_Gap != 0)
                {
                    Aim_Rx.Y_Speed = ( Aim_Rx.Aim_Yaw_Now   - Aim_Rx.Aim_Yaw_Last)   / Aim_Rx.Rx_Time_Gap / 1000;
                    Aim_Rx.P_Speed = ( Aim_Rx.Aim_Pitch_Now - Aim_Rx.Aim_Pitch_Last) / Aim_Rx.Rx_Time_Gap / 1000;
                }
                else
                {
                    Aim_Rx.Y_Speed = 0;
                    Aim_Rx.P_Speed = 0;
                }

                /* 卡尔曼滤波器 */
                KalmanII_Filter(&Y_Kalman, Aim_Rx.Aim_Yaw_Now,   Aim_Rx.Y_Speed);
                KalmanII_Filter(&P_Kalman, Aim_Rx.Aim_Pitch_Now, Aim_Rx.P_Speed);

                /* 设定阈值（防止视觉出现错误数据） */
                if(ABS(Aim_Rx.Yaw_Angle) <= 20  && ABS(Aim_Rx.Pitch_Angle) <= 20)
                {
//                    /* 获得自瞄期望角度（位置+速度*K，有一定预测） */
//                    Aim_Ref.Yaw  =  Y_Kalman.kalman.filtered_value[0] + Y_Kalman.kalman.filtered_value[1] * 0.10f;
//                    Aim_Ref.Pitch = P_Kalman.kalman.filtered_value[0] + P_Kalman.kalman.filtered_value[1] * 0.00f;
                    /* 获得自瞄期望角度(无Kalmanll_Filter) */
                    Aim_Ref.Yaw = Aim_Rx.Yaw_Send_Angle[Aim_Rx.Rx_ID]     + Aim_Rx.Yaw_Angle;
                    Aim_Ref.Pitch = Aim_Rx.Pitch_Send_Angle[Aim_Rx.Rx_ID] + Aim_Rx.Pitch_Angle;
                }

                /* ____ ,启动 ( 防呆/斜眼笑 ) */
                Aim_Rx.aim_start = 1;

                /* 自瞄时遥控器不可控 */
                Gyro_Ref.Yaw   = IMU.EulerAngler.ContinuousYaw;
                Gyro_Ref.Pitch = IMU.EulerAngler.Pitch;

            }     /* 相机丢失目标重置Kalman_Filter */
            else if(Aim_Rx.Tracker_Status == 0)
            {
                /* 关闭自瞄，估计值值状态更新 */
                Aim_Rx.aim_start = 0;
                Y_Kalman.xhat_data[0] = IMU.EulerAngler.ContinuousYaw; Y_Kalman.xhat_data[1] = 0;
                P_Kalman.xhat_data[0] = IMU.EulerAngler.Pitch;         P_Kalman.xhat_data[1] = 0;
                Y_Kalman.P_data[0] = 2; Y_Kalman.P_data[3] = 2;        
                P_Kalman.P_data[0] = 2; P_Kalman.P_data[3] = 2;        
            }
            /* 通过ID判断是否更新Kalmanll_Filter */
            Last_ID = Aim_Rx.Rx_ID;
        }
    }
    else
    {
        /* 关闭自瞄，估计值值状态更新 */
        Aim_Rx.aim_start = 0;
        Y_Kalman.xhat_data[0] = IMU.EulerAngler.ContinuousYaw; Y_Kalman.xhat_data[1] = 0;
        P_Kalman.xhat_data[0] = IMU.EulerAngler.Pitch;         P_Kalman.xhat_data[1] = 0;
        Y_Kalman.P_data[0] = 2; Y_Kalman.P_data[3] = 2;
        P_Kalman.P_data[0] = 2; P_Kalman.P_data[3] = 2;
    }
}

/* 虚拟串口PC通信接收回调函数 */
void VCOMM_CallBack(uint8_t fun_code, uint16_t id, uint8_t *data, uint8_t len)
{
      static uint64_t Last_Time;
      Aim_Rx.fun_code_rx = fun_code;
      Aim_Rx.id_rx       = id;
      if ( Aim_Rx.id_rx == 0 && Aim_Rx.fun_code_rx == 1 )
      {
            /* 获得时间间隔*/
            Aim_Rx.Rx_Time_Gap = xTaskGetTickCount() - Last_Time;
            Last_Time          = xTaskGetTickCount();

            /* 记录上次云台角度 */
            Aim_Rx.Aim_Yaw_Last   = Aim_Rx.Aim_Yaw_Now;
            Aim_Rx.Aim_Pitch_Last = Aim_Rx.Aim_Pitch_Now;

            /* 转存数据 */
            memcpy(&Aim_Rx_infopack, data, sizeof(Aim_Rx_info));
            Aim_Rx.Yaw_Angle      = -Aim_Rx_infopack.yaw   + Aim_Rx.Yaw_Angle_Offset;
            Aim_Rx.Pitch_Angle    = -Aim_Rx_infopack.pitch + Aim_Rx.Pitch_Angle_Offset;
            Aim_Rx.Distance       =  Aim_Rx_infopack.distance;
            Aim_Rx.Shoot_Flag     =  Aim_Rx_infopack.aim_shoot;
            Aim_Rx.Tracker_Status =  Aim_Rx_infopack.tracker_status;
            Aim_Rx.Rx_ID          =  Aim_Rx_infopack.rx_id;
      }
      
      if (Aim_Rx.id_rx == 0 && Aim_Rx.fun_code_rx == 0 )
      {
          /* 校准本地时间戳 */
          memcpy(&Aim_Rx.StandardTimeStamp, data, sizeof(Aim_Rx.StandardTimeStamp));
          Aim_Rx.TimeStamp_setoff = Aim_Rx.StandardTimeStamp - xTaskGetTickCount();
          Last_Time               = xTaskGetTickCount();
          Aim_Rx.Rx_ID            = 0;
      }
      Feed_Dog(&PC_Dog);
}

/* 虚拟串口PC通信错误回调函数 */
void VCOMM_Error_CallBack(uint8_t *data, uint8_t len) {
    UNUSED(data);
    UNUSED(len);
}
