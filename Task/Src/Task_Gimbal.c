/*!
 * @file    Task_Gimbal.c
 * @date    2024-1-1   
 * @brief   云台控制任务
 */
#include "Task_Gimbal.h"

/* 云台控制主任务 */
void Task_Gimbal(void *pvParameters)
{
    static portTickType currentTime;
    for (;;)
    {
        currentTime = xTaskGetTickCount();
        if (systemState == SYSTEM_STARTING)
        {
            Median_Init();
            #if !GIMBAL_RUN
            systemState = SYSTEM_RUNNING;
            #endif
        }
        else if (systemState == SYSTEM_RUNNING)
        {
            #if GIMBAL_RUN
            if ((Gimbal_State[PITCH] == Device_Online) && (Gimbal_State[YAW] == Device_Online))
            {
            #endif
                if (RemoteMode == REMOTE_INPUT)
                    Gimbal_Rc_Ctrl();
                else if (RemoteMode == KEY_MOUSE_INPUT)
                    Gimbal_Key_Ctrl();
                else
                    Gimbal_Stop();
                if (RemoteMode != STOP)
                {
                    if(PC_State == Device_Online)
                        Aim_Control();
                    Gimbal_Move();
                }
#if GIMBAL_RUN
                MotorSend(&hcan2, 0x1FF, Can2Send_Gimbal);
            }
            else
                Gimbal_Close();
#endif
        }
        vTaskDelayUntil(&currentTime, 2);
    }
}

/* 云台归中 */
void Median_Init()
{
    static uint16_t tim = 0;
    static float Expect_PitchInit = 0;
    static float Expect_YawInit = 0;

    if ((Gimbal_State[PITCH] == Device_Online) && (Gimbal_State[YAW] == Device_Online))
        tim++;
    else
        tim = 0;
    if (tim < 10)
    {
        /* 获得归中位置(前后就近归中) */
        if ( (Gimbal_Motor[YAW].MchanicalAngle <= Yaw_Mid_Right) || (Gimbal_Motor[YAW].MchanicalAngle >= Yaw_Mid_Left) )
        {
            MidMode = FRONT;
        }
        else
        {
            MidMode = BACK;
        }
        Expect_PitchInit = QuickCentering( Gimbal_Motor[PITCH].MchanicalAngle, Pitch_Mid );
    }
    else
    {
        /* 快速归中未加斜坡（前后就近归中） */
        if (MidMode == FRONT)
            Expect_YawInit = QuickCentering( Gimbal_Motor[YAW].MchanicalAngle, Yaw_Mid_Front );
        else
            Expect_YawInit = QuickCentering( Gimbal_Motor[YAW].MchanicalAngle, Yaw_Mid_Back );

        Expect_PitchInit = QuickCentering( Gimbal_Motor[PITCH].MchanicalAngle, Pitch_Mid );
        
        /* Yaw轴归中 */
        PID_Control_Smis( Gimbal_Motor[YAW].MchanicalAngle, Expect_YawInit, &Gimbal_Place_PIDS[YAW][Init], Gimbal_Motor[YAW].Speed );
        PID_Control( Gimbal_Motor[YAW].Speed, Gimbal_Place_PIDS[YAW][Init].pid_out, &Gimbal_Speed_PID[YAW][Init] );
        limit( Gimbal_Speed_PID[YAW][Init].pid_out, GM6020_LIMIT, -GM6020_LIMIT );
        
        /* Pitch轴归中 */
        PID_Control_Smis( Gimbal_Motor[PITCH].MchanicalAngle, Expect_PitchInit, &Gimbal_Place_PIDS[PITCH][Init], Gimbal_Motor[PITCH].Speed );
        PID_Control( Gimbal_Motor[PITCH].Speed, Gimbal_Place_PIDS[PITCH][Init].pid_out, &Gimbal_Speed_PID[PITCH][Init] );
        limit( Gimbal_Speed_PID[PITCH][Init].pid_out, GM6020_LIMIT, -GM6020_LIMIT );

        Can2Send_Gimbal[YAW] = (int16_t)Gimbal_Speed_PID[YAW][Init].pid_out;
        Can2Send_Gimbal[PITCH] = (int16_t)Gimbal_Speed_PID[PITCH][Init].pid_out;

#if GIMBAL_RUN
        MotorSend(&hcan2, 0x1FF, Can2Send_Gimbal);
#endif
    }
    if (tim >= 800)
    {
        tim = 0;
        systemState = SYSTEM_RUNNING;

        /* 记录角度 */
        Mech_Ref.Yaw = Gimbal_Motor[YAW].MchanicalAngle; // 机械模式  Y轴固定
        Mech_Ref.Pitch = Gimbal_Motor[PITCH].Angle;      //            P轴用电机连续化角度控制
        Gyro_Ref.Yaw = IMU.EulerAngler.ContinuousYaw;    // 陀螺仪模式  Y轴用IMU.ContinueYaw控制
        Gyro_Ref.Pitch = IMU.EulerAngler.Pitch;          //                  P轴用IMU.Pitch控制
        Aim_Rx.Yaw_Angle = 0;                           //视觉自瞄角
        Aim_Rx.Pitch_Angle = 0;
        Gimbal_Ramp_Angle.Yaw = Gimbal_Motor[YAW].Angle;        //归中斜坡（未使用）
        Gimbal_Ramp_Angle.Pitch = Gimbal_Motor[PITCH].Angle;    //归中斜坡
        Gimbal_increase[YAW][Change_RC] = 0;    //控制位置增量
        Gimbal_increase[PITCH][Change_RC] = 0;
        Gimbal_increase[YAW][Change_KEY] = 0;
        Gimbal_increase[PITCH][Change_KEY] = 0;
    }
}

/* 遥控器模式（云台） */
void Gimbal_Rc_Ctrl()
{
    /* 如果陀螺仪在线就用陀螺仪，陀螺仪离线就用电机角度控制 */
    if (IMU_State == Device_Online)
    {
        /* 陀螺仪模式 */
        CtrlMode = GYRO_MODE;

        /* Yaw轴 */
        Gimbal_increase[YAW][Change_RC] = Key_ch[2] * A_Y;
        Gyro_Ref.Yaw -= (Gimbal_increase[YAW][Change_RC]);

        /* Pitch轴 */
        Gimbal_increase[PITCH][Change_RC] = -Key_ch[3] * A_P;
        Gyro_Ref.Pitch -= Gimbal_increase[PITCH][Change_RC];
        limit(Gyro_Ref.Pitch, IMU_UP_limit, IMU_DOWN_limit);

        /* 记录角度 */
        Mech_Ref.Pitch = Gimbal_Motor[PITCH].Angle;
    }
    else
    {
        /* 陀螺仪离线机械模式（底盘强制为正常底盘） */
        CtrlMode = MECH_MODE;
        ChassisAction = CHASSIS_NORMAL;

        /* Yaw轴机械角度固定(Yaw电机不动，控制底盘旋转) */
        if (MidMode == FRONT)
            Mech_Ref.Yaw = Yaw_Mid_Front;
        else
            Mech_Ref.Yaw = Yaw_Mid_Back;

        /* Yaw轴转向是控制底盘旋转 */

        /* Pitch轴电机机械角度控制 */
        Gimbal_increase[PITCH][Change_RC] = -Key_ch[3] * M_P;
        Mech_Ref.Pitch -= Gimbal_increase[PITCH][Change_RC];
        limit(Mech_Ref.Pitch, P_UP_limit, P_DOWN_limit);

        /* 记录角度 */
        Gyro_Ref.Yaw = IMU.EulerAngler.ContinuousYaw;
        Gyro_Ref.Pitch = IMU.EulerAngler.Pitch;
    }
}

/* 键鼠模式（云台） */
void Gimbal_Key_Ctrl()
{
    static char Key_V = 0;
    /* 如果陀螺仪在线就用陀螺仪，陀螺仪离线就用电机机械角度控制 */
    if ( IMU_State == Device_Online )
    {
        /* 陀螺仪模式 */
        CtrlMode = GYRO_MODE;

        /* Yaw轴 */
        Gimbal_increase[YAW][Change_KEY] = Mouse_ch[0] * A_Y_1;
        Gyro_Ref.Yaw -= (Gimbal_increase[YAW][Change_KEY]);

        /* Yaw轴快速调头180° */
        if (RC_CtrlData.key.V == 1 && Key_V == 0)
        {
            Gyro_Ref.Yaw += 180;
            Key_V = 1;
            if (MidMode == FRONT)
                MidMode = BACK;
            else
                MidMode = FRONT;
        }
        if (RC_CtrlData.key.V == 0)
            Key_V = 0;

        /* Pitch轴 */
        Gimbal_increase[PITCH][Change_KEY] = Mouse_ch[1] * A_P;
        Gyro_Ref.Pitch -= Gimbal_increase[PITCH][Change_KEY];
        limit(Gyro_Ref.Pitch, IMU_UP_limit, IMU_DOWN_limit);
        
        /* 记录角度 */
        Mech_Ref.Pitch = Gimbal_Motor[PITCH].Angle;
    }
    else
    {
        /* 陀螺仪离线机械模式（底盘强制为正常底盘） */
        CtrlMode = MECH_MODE;
        ChassisAction = CHASSIS_NORMAL;

        /* Yaw轴机械角度固定(Yaw电机不动，控制底盘旋转) */
        if (MidMode == FRONT)
            Mech_Ref.Yaw = Yaw_Mid_Front;
        else
            Mech_Ref.Yaw = Yaw_Mid_Back;

        /* Yaw轴控制底盘旋转 */
        if (RC_CtrlData.key.Q == 1)
            Key_ch[2] = -1;
        else if (RC_CtrlData.key.E == 1)
            Key_ch[2] = 1;
        else
            Key_ch[2] = 0;

        /* Pitch轴电机角度控制 */
        Gimbal_increase[PITCH][Change_KEY] = Key_ch[3] * A_P;
        Mech_Ref.Pitch -= Gimbal_increase[PITCH][Change_KEY];
        limit(Mech_Ref.Pitch, P_UP_limit, P_DOWN_limit);

        /* 记录角度 */
        Gyro_Ref.Yaw = IMU.EulerAngler.ContinuousYaw;
        Gyro_Ref.Pitch = IMU.EulerAngler.Pitch;
    }
}

/* 云台PID计算函数 */
void Gimbal_Move()
{
    static PTZAngle_Ref_t Angle_Ref;
    
    /* 不同模式下云台期望角获取 */
    if (CtrlMode == GYRO_MODE)
    {
        if(Aim_Rx.aim_start)
        {
            /* 陀螺仪模式(自瞄) */
            Angle_Ref.Yaw = Aim_Ref.Yaw;
            Angle_Ref.Pitch = Aim_Ref.Pitch;
        }
        else
        {
            /* 陀螺仪模式(无自瞄) */
            Angle_Ref.Yaw = Gyro_Ref.Yaw;
            Angle_Ref.Pitch = Gyro_Ref.Pitch;
        }
            /* Yaw轴 */
        PID_Control_Smis(IMU.EulerAngler.ContinuousYaw, Angle_Ref.Yaw, &Gimbal_Place_PIDS[YAW][Gyro], IMU.AngularVelocity.Z);
        PID_Control(IMU.AngularVelocity.Z, Gimbal_Place_PIDS[YAW][Gyro].pid_out, &Gimbal_Speed_PID[YAW][Gyro]);
        limit(Gimbal_Speed_PID[YAW][Gyro].pid_out, GM6020_LIMIT, -GM6020_LIMIT);
        Can2Send_Gimbal[YAW] = (int16_t)Gimbal_Speed_PID[YAW][Gyro].pid_out;
        
            /* Pitch轴 */
        PID_Control_Smis(IMU.EulerAngler.Pitch, Angle_Ref.Pitch, &Gimbal_Place_PIDS[PITCH][Gyro], IMU.AngularVelocity.Y);
        PID_Control(IMU.AngularVelocity.Y, Gimbal_Place_PIDS[PITCH][Gyro].pid_out, &Gimbal_Speed_PID[PITCH][Gyro]);
        limit(Gimbal_Speed_PID[PITCH][Gyro].pid_out, GM6020_LIMIT, -GM6020_LIMIT);
        Can2Send_Gimbal[PITCH] = (int16_t)Gimbal_Speed_PID[PITCH][Gyro].pid_out;
    }
    else
    {
        /* 机械模式 */
        Angle_Ref.Yaw = QuickCentering(Gimbal_Motor[YAW].MchanicalAngle, Mech_Ref.Yaw);
        Angle_Ref.Pitch = Mech_Ref.Pitch;
        
        /* Yaw轴 */
        PID_Control_Smis(Gimbal_Motor[YAW].MchanicalAngle, Angle_Ref.Yaw, &Gimbal_Place_PIDS[YAW][Mech], Gimbal_Motor[YAW].Speed);
        PID_Control(Gimbal_Motor[YAW].Speed, Gimbal_Place_PIDS[YAW][Mech].pid_out, &Gimbal_Speed_PID[YAW][Mech]);
        limit(Gimbal_Speed_PID[YAW][Mech].pid_out, GM6020_LIMIT, -GM6020_LIMIT);
        Can2Send_Gimbal[YAW] = (int16_t)Gimbal_Speed_PID[YAW][Mech].pid_out;
        
        /* Pitch轴 */
        PID_Control_Smis(Gimbal_Motor[PITCH].Angle, Angle_Ref.Pitch, &Gimbal_Place_PIDS[PITCH][Mech], Gimbal_Motor[PITCH].Speed);
        PID_Control(Gimbal_Motor[PITCH].Speed, Gimbal_Place_PIDS[PITCH][Mech].pid_out, &Gimbal_Speed_PID[PITCH][Mech]);
        limit(Gimbal_Speed_PID[PITCH][Mech].pid_out, GM6020_LIMIT, -GM6020_LIMIT);
        Can2Send_Gimbal[PITCH] = (int16_t)Gimbal_Speed_PID[PITCH][Mech].pid_out;
    }
}
