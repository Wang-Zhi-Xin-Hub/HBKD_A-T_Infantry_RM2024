#include "Task_Chassis_down.h"
/* 
    TODO：底盘跟随看效果类似前馈算法(如果PID也可以就算了)
*/
/* 底盘控制（下板通信）主任务 */
void Task_Chassis_down(void *pvParameters)
{
    static portTickType currentTime;
    for (;;)
    {
        currentTime = xTaskGetTickCount();

        if (systemState != SYSTEM_RUNNING){
            Chassis_Stop();
        } else {
            if (Remote_State == Device_Online){
                if (RemoteMode == REMOTE_INPUT)
                    Chassis_RC_Ctrl();
                else if (RemoteMode == KEY_MOUSE_INPUT)
                    Chassis_Key_Ctrl();
                else
                    Chassis_Stop();
                if (RemoteMode != STOP)
                    Chassis_Drive();
#if CHASSIS_RUN
                CAN_Send_StdDataFrame(&hcan2, 0x110, (uint8_t *)&Communication_Speed_Tx);
#endif
            }else
                Chassis_Close();
        }
        vTaskDelayUntil(&currentTime, 1);
    }
}

/* 遥控器模式（底盘） */
void Chassis_RC_Ctrl()
{
    /* 遥控器调试机器人 */
    switch (RC_CtrlData.rc.s1){
    case 1:
        ChassisAction = CHASSIS_NORMAL;
        AimAction = AIM_AUTO;
        if(ShootAction != SHOOT_STUCKING && AimAction != AIM_AUTO)
            ShootAction = SHOOT_NORMAL;
    break;

    case 3:
        ChassisAction = CHASSIS_NORMAL;
        AimAction = AIM_AID;
        if(ShootAction != SHOOT_STUCKING)
            ShootAction = SHOOT_READY;
    break;

    case 2:
        ChassisAction = CHASSIS_NORMAL;
        AimAction = AIM_STOP;
        ShootAction = SHOOT_STOP;
    break;
    }
    
    /*
    ChassisAction：                     ShootAction：                             AimAction：
    CHASSIS_NORMAL //普通底盘(调试用)   SHOOT_STOP     //停止发射                 AIM_STOP  //关闭自瞄
    CHASSIS_SPIN   //小陀螺模式	        SHOOT_READY    //准备发射（摩擦轮启动）   AIM_AID   //自瞄不自动射击
    CHASSIS_FOLLOW //底盘跟随模式       SHOOT_NORMAL   //单发                     AIM_AUTO  //自瞄+自动射击
    CHASSIS_RADAR  //雷达导航           SHOOT_RUNNING  //速射(单发超过0.3s变连发)
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

    /* R键切换底盘模式 */
    if (RC_CtrlData.key.R == 1 && Key_R == 0){
        if (ChassisAction == CHASSIS_FOLLOW)
            ChassisAction = CHASSIS_SPIN;
        else
            ChassisAction = CHASSIS_FOLLOW;
        Key_R = 1;
    }
    if (RC_CtrlData.key.R == 0)
        Key_R = 0;
    /* 快速转向后不松开V键将直接开始小陀螺，松开则停 */
    if (RC_CtrlData.key.V == 1 && Key_V == 0){
        ChassisAction = CHASSIS_SPIN;
        Key_V = 1;
    } else {
        if (RC_CtrlData.key.V == 0 && Key_V == 1){
            ChassisAction = CHASSIS_FOLLOW;
            Key_V = 0;
        }
    }
}
/* 底盘驱动 */
void Chassis_Drive()
{
    if (CtrlMode == GYRO_MODE){
        if(ChassisAction == CHASSIS_FOLLOW)
            Chassis_Follow();
        else if (ChassisAction == CHASSIS_SPIN)
            Variable_Speed_Gyroscope();
        else
            Communication_Speed_Tx.Chassis_Speed.rotate_ref = 0;
    } else {
        if(ChassisAction == CHASSIS_RADAR)
            Communication_Speed_Tx.Chassis_Speed.rotate_ref =  - Radar_Chassis_Speed.rotate_ref * STD_Omega;
        else
            Communication_Speed_Tx.Chassis_Speed.rotate_ref = Key_ch[2] * STD_Omega;  //IMU离线只能控制底盘旋转转向
    }
    /* 底盘补偿 */
    Chassis_Offset();
}
/* 底盘跟随 */
void Chassis_Follow()
{
    static uint16_t Follow_Speed_MAX = 8000;
    static int16_t  Yaw_Mid = FRONT;
    /* 底盘跟随（直接判断Yaw轴机械角度判断最近归中位置） */
    if ( (Gimbal_Motor[YAW].MchanicalAngle <= Yaw_Mid_Left) || (Gimbal_Motor[YAW].MchanicalAngle >= Yaw_Mid_Right) ){
        Yaw_Mid = Yaw_Mid_Front; 
        MidMode = FRONT;
    } else{
        Yaw_Mid = Yaw_Mid_Back;
        MidMode = BACK;
    }
    PID_Control_Smis(Gimbal_Motor[YAW].MchanicalAngle, QuickCentering(Gimbal_Motor[YAW].MchanicalAngle, Yaw_Mid), &Chassis_Speed_PIDS, IMU.AngularVelocity.Z);
    PID_Control(Gimbal_Motor[YAW].Speed, Chassis_Speed_PIDS.pid_out, &Chassis_Speed_PID);
    Communication_Speed_Tx.Chassis_Speed.rotate_ref = Chassis_Speed_PID.pid_out + FeedForward_Calc(&Chassis_FF);
    limit(Communication_Speed_Tx.Chassis_Speed.rotate_ref, Follow_Speed_MAX, -Follow_Speed_MAX);
    #if !GIMBAL_RUN
        Communication_Speed_Tx.Chassis_Speed.rotate_ref = 0;  //云台只有在IMU的控制才能实现底盘跟随
    #endif
}

/* 底盘补偿计算 */
void Chassis_Offset()
{
    static float Level_Gain, chassis_offset;
    static int16_t forward_back_ref = 0, left_right_ref = 0;
    Level_Gain = 4.0f + Referee_data_Rx.robot_level * 0.2;                            // 等级增益
    chassis_offset = (Gimbal_Motor[YAW].MchanicalAngle - Yaw_Mid_Front) / 1303.80f;   // 底盘补偿角

    if (RC_CtrlData.key.Shift){
        left_right_ref   = Key_ch[0] * STD_Speed * Level_Gain * 2.0;
        forward_back_ref = Key_ch[1] * STD_Speed * Level_Gain * 2.0;
    } else{
        left_right_ref   = Key_ch[0] * STD_Speed * Level_Gain;
        forward_back_ref = Key_ch[1] * STD_Speed * Level_Gain;
    }
    
    /* 雷达导航模式 */
    if(ChassisAction == CHASSIS_RADAR){
        forward_back_ref =    Radar_Chassis_Speed.forward_back_ref * STD_Speed * Level_Gain ;
        left_right_ref   =  - Radar_Chassis_Speed.left_right_ref   * STD_Speed * Level_Gain ;
    } else {
        Radar_Chassis_Speed.forward_back_ref = 0;
        Radar_Chassis_Speed.left_right_ref   = 0;
        Radar_Chassis_Speed.rotate_ref       = 0;
    }
    
    /* Y轴在线才能解算补偿 */
    if(Gimbal_State[YAW] == Device_Online){
        Communication_Speed_Tx.Chassis_Speed.forward_back_ref =   forward_back_ref * cosf(  chassis_offset)//底盘解算(小陀螺模式也能正常移动)
                                                                + left_right_ref * sinf(  chassis_offset);
        Communication_Speed_Tx.Chassis_Speed.left_right_ref   =  forward_back_ref * sinf( -chassis_offset) 
                                                                + left_right_ref * cosf( -chassis_offset);
    } else{
        Communication_Speed_Tx.Chassis_Speed.forward_back_ref =  forward_back_ref;
        Communication_Speed_Tx.Chassis_Speed.left_right_ref   =  left_right_ref;
    }
}

/* 变速小陀螺 */
void Variable_Speed_Gyroscope()
{
    static uint16_t Follow_Speed_MAX = 6000;
    static float time  = 0;  // 时间比例（变速区间0.75 - 1.30 每2ms变0.005f）
    static float angle = 0;  // 角度比例
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
        Communication_Speed_Tx.Chassis_Speed.rotate_ref = (2000 + Referee_data_Rx.robot_level * 150) * Variable_Speed_K;
    else
        Communication_Speed_Tx.Chassis_Speed.rotate_ref = (3000 + Referee_data_Rx.robot_level * 250) * Variable_Speed_K;
    
    limit(Communication_Speed_Tx.Chassis_Speed.rotate_ref, Follow_Speed_MAX, - Follow_Speed_MAX);
}
