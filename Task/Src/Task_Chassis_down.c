/**
 * @file     Task_Chassis_down.c
 * @date     2023-12-1
 * @brief    底盘控制（下板通信）任务
 */
#include "Task_Chassis_down.h"

/* 底盘控制（下板通信）主任务（底盘期望速度ID：0x110 各机构模式反馈（UI）ID：0x120） */
void Task_Chassis_down(void *pvParameters)
{
    static portTickType currentTime;
    for (;;)
    {
        currentTime = xTaskGetTickCount(); // 获取当前时间
        if (systemState == SYSTEM_STARTING)
        {
            Chassis_Stop();
            ChassisAction = CHASSIS_FOLLOW;
        }
        else if (systemState == SYSTEM_RUNNING)
        {
            if (Remote_State == Device_Online)
            {
                if (RemoteMode == REMOTE_INPUT)
                    Chassis_RC_Ctrl();
                else if (RemoteMode == KEY_MOUSE_INPUT)
                    Chassis_Key_Ctrl();
                else
                    Chassis_Stop();

                if (RemoteMode != STOP)
                {
                    if (CtrlMode == GYRO_MODE)
                    {
                        if (ChassisAction == CHASSIS_FOLLOW)
                            Chassis_Follow();
                        else if (ChassisAction == CHASSIS_SPIN)
                            Variable_Speed_Gyroscope();
                        else
                            Communication_Speed_Tx.Chassis_Speed.rotate_ref = 0;
                    }
                    else
                        Communication_Speed_Tx.Chassis_Speed.rotate_ref = Key_ch[2] * CHASSIS_Speed_R * 2.4f;  //IMU离线只能控制底盘旋转转向
                }
                Chassis_Move();

#if CHASSIS_RUN
                CAN_Send_StdDataFrame(&hcan1, 0x110, (uint8_t *)&Communication_Speed_Tx);
#endif
            }
            else
                Chassis_Close();
        }
        /* 各机构状态（发给下板动态UI显示） */
        Send_UI_State();        
        vTaskDelayUntil(&currentTime, 2);
    }
}

/* 遥控器模式（底盘） */
void Chassis_RC_Ctrl()
{
    /* 遥控器调试机器人（Shoot.c中需注释） */
    switch (RC_CtrlData.rc.s1)
    {
    case 1:
        ChassisAction = CHASSIS_SPIN;
        AimAction = AIM_STOP;
        if(ShootAction != SHOOT_STUCKING && AimAction != AIM_AUTO)
        ShootAction = SHOOT_STOP;
    break;

    case 3:
        ChassisAction = CHASSIS_SPIN;
        AimAction = AIM_STOP;
        if(ShootAction != SHOOT_STUCKING)
        ShootAction = SHOOT_STOP;
    break;

    case 2:
        ChassisAction = CHASSIS_FOLLOW;
        AimAction = AIM_STOP;
        ShootAction = SHOOT_STOP;
    break;
    }
    /*
    ChassisAction：                     ShootAction：                             AimAction：
    CHASSIS_NORMAL //普通底盘(调试用)   SHOOT_STOP     //停止发射                 AIM_STOP  //关闭自瞄
    CHASSIS_SPIN   //小陀螺模式	        SHOOT_READY    //准备发射（摩擦轮启动）   AIM_AID   //辅瞄不自动射击
    CHASSIS_FOLLOW //底盘跟随模式       SHOOT_NORMAL   //单发                     AIM_AUTO  //自瞄+自动射击
                                        SHOOT_RUNNING  //速射(单发超过0.3s变连发)
                                        SHOOT_STUCKING //卡弹退弹中
    */
}

/* 键鼠模式（底盘） */
void Chassis_Key_Ctrl()
{
    static char Key_R = 0, Key_V = 0;
    /* WS前后 */
    if (RC_CtrlData.key.W)
        Key_ch[1] = 1;
    else if (RC_CtrlData.key.S)
        Key_ch[1] = -1;
    else
        Key_ch[1] = 0;
    /* AD左右 */
    if (RC_CtrlData.key.A)
        Key_ch[0] = -1;
    else if (RC_CtrlData.key.D)
        Key_ch[0] = 1;
    else
        Key_ch[0] = 0;
    /* Shift加速 */
    if (RC_CtrlData.key.Shift)
        Communication_Speed_Tx.Shift_flag = 1;
    else
        Communication_Speed_Tx.Shift_flag = 0;
    /* R键切换底盘模式 */
    if (RC_CtrlData.key.R == 1 && Key_R == 0)
    {
        if (ChassisAction == CHASSIS_FOLLOW)
            ChassisAction = CHASSIS_SPIN;
        else
            ChassisAction = CHASSIS_FOLLOW;
        Key_R = 1;
    }
    if (RC_CtrlData.key.R == 0)
        Key_R = 0;
    /* 快速转向后不松开V键将直接开始小陀螺，松开则停 */
    if (RC_CtrlData.key.V == 1 && Key_V == 0)
    {
        ChassisAction = CHASSIS_SPIN;
        Key_V = 1;
    }
    else if (RC_CtrlData.key.V == 0 && Key_V == 1)
    {
        ChassisAction = CHASSIS_FOLLOW;
        Key_V = 0;
    }
}

/* 底盘跟随 */
void Chassis_Follow()
{
    /* 底盘跟随（直接判断Yaw轴机械角度判断最近归中位置） */
    if ( (Gimbal_Motor[YAW].MchanicalAngle <= Yaw_Mid_Right) || (Gimbal_Motor[YAW].MchanicalAngle >= Yaw_Mid_Left) )
    {
        PID_Control_Smis(Gimbal_Motor[YAW].MchanicalAngle, QuickCentering(Gimbal_Motor[YAW].MchanicalAngle, Yaw_Mid_Front), &Chassis_Speed_PIDS, IMU.AngularVelocity.Z);
        PID_Control(Gimbal_Motor[YAW].Speed, Chassis_Speed_PIDS.pid_out, &Chassis_Speed_PID);
        Communication_Speed_Tx.Chassis_Speed.rotate_ref = Chassis_Speed_PID.pid_out;
        MidMode = FRONT;
    }
    else
    {
        PID_Control_Smis(Gimbal_Motor[YAW].MchanicalAngle, QuickCentering(Gimbal_Motor[YAW].MchanicalAngle, Yaw_Mid_Back), &Chassis_Speed_PIDS, IMU.AngularVelocity.Z);
        PID_Control(Gimbal_Motor[YAW].Speed, Chassis_Speed_PIDS.pid_out, &Chassis_Speed_PID);
        Communication_Speed_Tx.Chassis_Speed.rotate_ref = Chassis_Speed_PID.pid_out;
        MidMode = BACK;
    }
}

/* 底盘补偿计算 */
void Chassis_Move()
{
    static float Move_level_gain, chassis_offset;
    static int16_t forward_back_ref = 0, left_right_ref = 0;

    Move_level_gain = 2.0f + Referee_data_Rx.robot_level * 0.2;                      // 速度等级增益
    chassis_offset = (Gimbal_Motor[YAW].MchanicalAngle - Yaw_Mid_Front) / 1303.80f;  // 底盘补偿角

    if (RC_CtrlData.key.Shift == 1)
    {
        left_right_ref = Key_ch[0] * CHASSIS_Speed_H_Y * Move_level_gain;
        forward_back_ref = Key_ch[1] * CHASSIS_Speed_H_X * Move_level_gain;
    }
    else
    {
        left_right_ref = Key_ch[0] * CHASSIS_Speed_L_Y * Move_level_gain * 2.0f;
        forward_back_ref = Key_ch[1] * CHASSIS_Speed_L_X * Move_level_gain * 2.0f;
    }
    if(Gimbal_State[YAW] == Device_Online){
        /* 底盘补偿(小陀螺模式也能正常移动)*/
        Communication_Speed_Tx.Chassis_Speed.forward_back_ref =  forward_back_ref * cosf(  chassis_offset)  + left_right_ref * sinf(  chassis_offset);
        Communication_Speed_Tx.Chassis_Speed.left_right_ref   =  forward_back_ref * sinf( -chassis_offset)  + left_right_ref * cosf( -chassis_offset);
    }
    else
    {
        Communication_Speed_Tx.Chassis_Speed.forward_back_ref =  forward_back_ref;
        Communication_Speed_Tx.Chassis_Speed.left_right_ref   =  left_right_ref;
    }
}

/* 变速小陀螺 */
void Variable_Speed_Gyroscope()
{
    static float time = 0;  // 时间比例（变速区间0.75 - 1.30 每2ms变0.005f）
    static float angle = 0; // 角度比例
    static float Variable_Speed_K = 1.0f;
    static char add_time_flag = 1;

    // 按时间变速(time为比例)
    if (add_time_flag == 1)
        time += 0.005f;
    else
        time -= 0.005f;
    if (time >= 0.75f)
        add_time_flag = 0;
    else if (time <= -0.30f)
        add_time_flag = 1;

    // 按角度变速（头为前方，装甲板向前转的快，对角转的慢）（angle为比例）
    if (Gimbal_Motor[YAW].MchanicalAngle <= 7880 && Gimbal_Motor[YAW].MchanicalAngle >= 5830)
        angle = ABS(6855.0f - Gimbal_Motor[YAW].MchanicalAngle) / 2050.57f;
    if (Gimbal_Motor[YAW].MchanicalAngle <= 5830 && Gimbal_Motor[YAW].MchanicalAngle >= 3780)
        angle = ABS(4805.0f - Gimbal_Motor[YAW].MchanicalAngle) / 2050.57f;
    if (Gimbal_Motor[YAW].MchanicalAngle <= 3780 && Gimbal_Motor[YAW].MchanicalAngle >= 1730)
        angle = ABS(2755.0f - Gimbal_Motor[YAW].MchanicalAngle) / 2050.57f;
    if (Gimbal_Motor[YAW].MchanicalAngle <= 1730 && Gimbal_Motor[YAW].MchanicalAngle > 0)
        angle = ABS(705.0f - Gimbal_Motor[YAW].MchanicalAngle) / 2050.57f;
    if (Gimbal_Motor[YAW].MchanicalAngle <= 8191 && Gimbal_Motor[YAW].MchanicalAngle >= 7880)
        angle = ABS(705.0f + 8191.0f - Gimbal_Motor[YAW].MchanicalAngle) / 2050.57f;

    /* 最终变速比例 */
    Variable_Speed_K = 1.0f;

    /* 小陀螺有等级增益，有移动速度时减小 */
    if (Key_ch[0] || Key_ch[1])
        Communication_Speed_Tx.Chassis_Speed.rotate_ref = (3000 + Referee_data_Rx.robot_level * 150) * Variable_Speed_K;
    else
        Communication_Speed_Tx.Chassis_Speed.rotate_ref = (6000 + Referee_data_Rx.robot_level * 250) * Variable_Speed_K;
}

/* 发送机构状态 */
void Send_UI_State()
{
    Communication_Action_Tx.ChassisAction_Tx = ChassisAction;
    Communication_Action_Tx.ShootAction_Tx = ShootAction;
    Communication_Action_Tx.AimAction_Tx = AimAction;
    Communication_Action_Tx.CtrlMode_Tx = CtrlMode;
    Communication_Action_Tx.MidMode_Tx = MidMode;

    CAN_Send_StdDataFrame(&hcan1, 0x120, (uint8_t *)&Communication_Action_Tx);
}
