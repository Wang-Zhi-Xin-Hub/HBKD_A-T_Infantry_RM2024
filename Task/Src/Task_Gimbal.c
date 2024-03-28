#include "Task_Gimbal.h"
/* TODO: 键鼠慢速移动，PID死区阈值，变速积分*/
/* TODO:归中完成之后自己记录中值 */

#define DEBUG_IMU_P_RC 1    //用于调试P轴用IMU控制时的PID（只能遥控器控制）
/* 云台控制主任务 */
void Task_Gimbal(void *pvParameters)
{
    static portTickType currentTime;
    for (;;)
    {
        currentTime = xTaskGetTickCount();
        if (systemState != SYSTEM_RUNNING)
        {
            Median_Init();
            #if !GIMBAL_RUN
            systemState = SYSTEM_RUNNING;
            #endif
        }
        else
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
                    Aim_Control();
                    Gimbal_Drive();
                }
#if GIMBAL_RUN
                MotorSend(&hcan2, 0x1FF, Can2Send_Gimbal);
            }
            else
                Gimbal_Close();
#endif
        }
        vTaskDelayUntil(&currentTime, 1);
    }
}

/* 云台归中 */
void Median_Init()
{
    static uint16_t tim = 0;
    static float Expect_PitchInit = 0;
    static float Expect_YawInit = 0;

    if ((Gimbal_State [PITCH] == Device_Online) && (Gimbal_State [YAW] == Device_Online))
        tim++;
    else
        tim = 0;
    if (tim < 10)
    {
        /* 获得归中位置(前后就近归中) */
        if ( (Gimbal_Motor[YAW].MchanicalAngle <= Yaw_Mid_Left) || (Gimbal_Motor[YAW].MchanicalAngle >= Yaw_Mid_Right) )
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

        Expect_PitchInit   = QuickCentering( Gimbal_Motor[PITCH].MchanicalAngle, Pitch_Mid );
        
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
    if (tim >= 1200)
    {
        /* 记录角度 */
        Mech_Ref.Yaw = Gimbal_Motor[YAW].MchanicalAngle; // 机械模式  Y轴固定
        Mech_Ref.Pitch = Gimbal_Motor[PITCH].Angle;      //            P轴用电机连续化机械角度控制
        Gyro_Ref.Pitch = IMU.EulerAngler.Pitch;    // 陀螺仪模式  启用后
        Gyro_Ref.Yaw = IMU.EulerAngler.ContinuousYaw;    // 陀螺仪模式  Y轴用IMU.ContinueYaw控制
        Gimbal_increase[YAW][Change_RC] = 0;    //控制位置增量
        Gimbal_increase[PITCH][Change_RC] = 0;
        Gimbal_increase[YAW][Change_KEY] = 0;
        Gimbal_increase[PITCH][Change_KEY] = 0;
        
        tim = 0;
        systemState = SYSTEM_RUNNING;
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
        Gimbal_increase[YAW][Change_RC] = -Key_ch[2] * STD_Angle * 0.3f;
        Gyro_Ref.Yaw -= (Gimbal_increase[YAW][Change_RC]);  
        
        /* Pitch轴（IMU） */
        #if DEBUG_IMU_P_RC
        Gimbal_increase[PITCH][Change_RC] = Key_ch[3] * STD_Angle * 0.3f;
        Gyro_Ref.Pitch -= Gimbal_increase[PITCH][Change_RC];
        limit(Gyro_Ref.Pitch, 30, -20);
        #endif
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
        /* 记录角度 */
        Gyro_Ref.Yaw = IMU.EulerAngler.ContinuousYaw;
    }
    #if !DEBUG_IMU_P_RC
    /* Pitch轴电机机械角度控制 */
    Gimbal_increase[PITCH][Change_RC] = Key_ch[3] * STD_MAngle * 0.3f;
    Mech_Ref.Pitch += Gimbal_increase[PITCH][Change_RC];
    limit(Mech_Ref.Pitch, Pitch_Mid + P_ADD_limit, Pitch_Mid - P_LOSE_limit);
    #endif
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
        Gimbal_increase[YAW][Change_KEY] = Mouse_ch[0] * STD_Angle * 0.3;
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
        
        /* 记录角度 */
        Gyro_Ref.Yaw = IMU.EulerAngler.ContinuousYaw;
    }
    
    /* Pitch轴电机角度控制 */
    Gimbal_increase[PITCH][Change_KEY] = Key_ch[3] * STD_MAngle * 0.3;
    Mech_Ref.Pitch += Gimbal_increase[PITCH][Change_KEY];
    limit(Mech_Ref.Pitch, Pitch_Mid + P_ADD_limit, Pitch_Mid - P_LOSE_limit);
}

/* 云台PID计算函数 */
void Gimbal_Drive()
{
    static PTZAngle_Ref_t Angle_Ref;
    
    /* 不同模式下云台期望角获取 */
    if (CtrlMode == GYRO_MODE)
    {
        if(Aim_Rx.aim_start)
        {
            /* 陀螺仪模式(自瞄) */
            Angle_Ref.Yaw   = Aim_Ref.Yaw;
            Angle_Ref.Pitch = Aim_Ref.Pitch;
            /* Pitch轴（IMU） */
            PID_Control_Smis(IMU.EulerAngler.Pitch, Angle_Ref.Pitch, &Gimbal_Place_PIDS[PITCH][Gyro], IMU.AngularVelocity.Y);
            PID_Control(IMU.AngularVelocity.Y, -Gimbal_Place_PIDS[PITCH][Gyro].pid_out, &Gimbal_Speed_PID[PITCH][Gyro]);
            limit(Gimbal_Speed_PID[PITCH][Gyro].pid_out, GM6020_LIMIT, -GM6020_LIMIT);
            Can2Send_Gimbal[PITCH] = (int16_t)Gimbal_Speed_PID[PITCH][Gyro].pid_out;
        }
        else
        {
            /* 陀螺仪模式(无自瞄) */
            Angle_Ref.Yaw   = Gyro_Ref.Yaw;
            #if DEBUG_IMU_P_RC
            Angle_Ref.Pitch = Gyro_Ref.Pitch;
            /* Pitch轴（IMU） */
            PID_Control_Smis(IMU.EulerAngler.Pitch, Angle_Ref.Pitch, &Gimbal_Place_PIDS[PITCH][Gyro], IMU.AngularVelocity.Y);
            PID_Control(IMU.AngularVelocity.Y, -Gimbal_Place_PIDS[PITCH][Gyro].pid_out, &Gimbal_Speed_PID[PITCH][Gyro]);
            limit(Gimbal_Speed_PID[PITCH][Gyro].pid_out, GM6020_LIMIT, -GM6020_LIMIT);
            Can2Send_Gimbal[PITCH] = (int16_t)Gimbal_Speed_PID[PITCH][Gyro].pid_out;
            #else
            Angle_Ref.Pitch = Mech_Ref.Pitch;
            /* Pitch轴（电机） */
            PID_Control_Smis(Gimbal_Motor[PITCH].Angle, Angle_Ref.Pitch, &Gimbal_Place_PIDS[PITCH][Mech], Gimbal_Motor[PITCH].Speed);
            PID_Control(Gimbal_Motor[PITCH].Speed, Gimbal_Place_PIDS[PITCH][Mech].pid_out, &Gimbal_Speed_PID[PITCH][Mech]);
            limit(Gimbal_Speed_PID[PITCH][Mech].pid_out, GM6020_LIMIT, -GM6020_LIMIT);
            Can2Send_Gimbal[PITCH] = (int16_t)Gimbal_Speed_PID[PITCH][Mech].pid_out;
            #endif

        }
        /* Yaw轴（IMU） */
        PID_Control_Smis(IMU.EulerAngler.ContinuousYaw, Angle_Ref.Yaw, &Gimbal_Place_PIDS[YAW][Gyro], IMU.AngularVelocity.Z);
        PID_Control(IMU.AngularVelocity.Z, -Gimbal_Place_PIDS[YAW][Gyro].pid_out, &Gimbal_Speed_PID[YAW][Gyro]);
        limit(Gimbal_Speed_PID[YAW][Gyro].pid_out, GM6020_LIMIT, -GM6020_LIMIT);
        Can2Send_Gimbal[YAW] = (int16_t)Gimbal_Speed_PID[YAW][Gyro].pid_out;
        
    }
    else
    {
        /* 机械模式 */
        Angle_Ref.Yaw = QuickCentering(Gimbal_Motor[YAW].MchanicalAngle, Mech_Ref.Yaw);
        Angle_Ref.Pitch = Mech_Ref.Pitch;
        
        /* Yaw轴（电机） */
        PID_Control_Smis(Gimbal_Motor[YAW].MchanicalAngle, Angle_Ref.Yaw, &Gimbal_Place_PIDS[YAW][Mech], Gimbal_Motor[YAW].Speed);
        PID_Control(Gimbal_Motor[YAW].Speed, Gimbal_Place_PIDS[YAW][Mech].pid_out, &Gimbal_Speed_PID[YAW][Mech]);
        limit(Gimbal_Speed_PID[YAW][Mech].pid_out, GM6020_LIMIT, -GM6020_LIMIT);
        Can2Send_Gimbal[YAW] = (int16_t)Gimbal_Speed_PID[YAW][Mech].pid_out;
        
        /* Pitch轴（电机） */
        PID_Control_Smis(Gimbal_Motor[PITCH].Angle, Angle_Ref.Pitch, &Gimbal_Place_PIDS[PITCH][Mech], Gimbal_Motor[PITCH].Speed);
        PID_Control(Gimbal_Motor[PITCH].Speed, Gimbal_Place_PIDS[PITCH][Mech].pid_out, &Gimbal_Speed_PID[PITCH][Mech]);
        limit(Gimbal_Speed_PID[PITCH][Mech].pid_out, GM6020_LIMIT, -GM6020_LIMIT);
        Can2Send_Gimbal[PITCH] = (int16_t)Gimbal_Speed_PID[PITCH][Mech].pid_out;
    }
}
