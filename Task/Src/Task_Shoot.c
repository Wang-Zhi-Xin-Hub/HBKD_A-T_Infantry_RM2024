#include "Task_Shoot.h"
/*  TODO:增益补偿
    TODO:裁判系统反馈信息：热量，弹速
    TODO:弹舱盖
*/
/* 射击控制主任务 */
void Task_Shoot(void *pvParameters)
{
    static portTickType currentTime;
	for(;;)
	{
        currentTime = xTaskGetTickCount(); 
		if(systemState != SYSTEM_RUNNING){
            AimAction = AIM_STOP;
			ShootAction = SHOOT_STOP;
        } else {
            if((Shoot_State[LEFT] == Device_Online) && (Shoot_State[RIGHT] == Device_Online) && (Pluck_State == Device_Online)){
                RemoteMode == REMOTE_INPUT ? Shoot_Rc_Ctrl() :
                RemoteMode == KEY_MOUSE_INPUT ? Shoot_Key_Ctrl() : Shoot_Stop();
                if(RemoteMode != STOP)
                    Shoot_Drive();
#if SHOOT_RUN
            MotorSend(&hcan1, 0X200, Can1Send_Shoot);
#endif
            } else
                Shoot_Close();
        }
        vTaskDelayUntil (&currentTime,1);
    }
}

/* 遥控器模式（发射机构） */
void Shoot_Rc_Ctrl()
{
    static uint16_t normal_time = 0;
    if(ShootAction == SHOOT_NORMAL){
        normal_time++;
        if(normal_time > 199){
            ShootAction = SHOOT_RUNNING;
            normal_time = 200;
        }
    } else
        normal_time = 0;
}
/* 键鼠模式（发射机构） */
void Shoot_Key_Ctrl()
{
    static char Key_r = 0;
    static uint16_t normal_time = 0;
    /* 检测鼠标是否按下 */
    if(RC_CtrlData.mouse.press_r == 1 && Key_r ==0){
        if(ShootAction == SHOOT_STOP)
            ShootAction = SHOOT_READY;
        else 
            ShootAction = SHOOT_STOP;
        Key_r = 1;
    }
    if (RC_CtrlData.mouse.press_r == 0)
        Key_r = 0;
    if(ShootAction != SHOOT_STOP){
        /* 长按鼠标左键进入连发模式，单击鼠标左键进行单发 */
        if(RC_CtrlData.mouse.press_l == 1){
            if(ShootAction != SHOOT_STUCKING)
            ShootAction = SHOOT_NORMAL;    
        } else {
            if(ShootAction != SHOOT_STUCKING)
            ShootAction = SHOOT_READY;
        }
    }
    if(ShootAction == SHOOT_NORMAL){
        normal_time++;
        if(normal_time > 199){
            ShootAction = SHOOT_RUNNING;
            normal_time = 200;
        }
    } else {
        normal_time = 0;
    }
}

/* 射击PID计算 */
void Shoot_Drive()
{
    /* 局部变量 */
    static float Angle_Target = 0;                    //!< @brief拨弹盘期望角度
    static float RAMP_Angle_Target = 0;               //!< @brief拨弹盘期望斜坡角度
    static uint8_t Add_Angle_Flag = 0;                //!< @brief拨弹盘角度+1标志位
    static float Stuck_Angle_Target = 0;              //!< @brief拨弹盘卡弹退弹期望角度
    static eShootAction Last_status = SHOOT_STOP;     //!< @brief记录卡弹前的状态
    static uint16_t  Stuck_time = 0;                  //!< @brief检测卡弹的时间
    static const uint16_t Stuck_thre = 200;           //!< @brief卡弹阈值，卡弹超过此时间便认为卡弹

    if(ShootAction == SHOOT_STOP){  //急停模式
        Shoot_Stop();   //发射机构急停
        Stuck_time = 0; //卡弹时间清零
        Angle_Target = Pluck_Motor.Angle_DEG;   //记录角度
        RAMP_Angle_Target = Pluck_Motor.Angle_DEG;
    } else { 
        /* 准备发射（摩擦轮启动）(17mm弹丸速度限制固定30m/s) */
        PID_Control(Shoot_Motor [LEFT].Speed, -SHOOT_SPEED, &Shoot_Speed_PID [LEFT]);
        PID_Control(Shoot_Motor [RIGHT].Speed, SHOOT_SPEED, &Shoot_Speed_PID [RIGHT]);
        limit(Shoot_Speed_PID[LEFT].pid_out, RM3508_LIMIT, -RM3508_LIMIT);
        limit(Shoot_Speed_PID[RIGHT].pid_out, RM3508_LIMIT, -RM3508_LIMIT);
        Can1Send_Shoot[1] = (int16_t)Shoot_Speed_PID [LEFT].pid_out;
        Can1Send_Shoot[2] = (int16_t)Shoot_Speed_PID [RIGHT].pid_out;
       if(Add_Angle_Flag == 1 && ShootAction == SHOOT_NORMAL){          //拨弹盘期望角度加一个弹丸位置
           Angle_Target = Pluck_Motor.Angle_DEG + PLUCK_MOTOR_ONE;      //拨弹盘期望角度 = 当前拨弹盘角度 + 一个弹丸的位置
           RAMP_Angle_Target = Pluck_Motor.Angle_DEG;
           Add_Angle_Flag = 0;
       }
       if(ShootAction == SHOOT_READY)
           Add_Angle_Flag = 1;
        if(ShootAction == SHOOT_NORMAL || ShootAction == SHOOT_READY){  //单发
           RAMP_Angle_Target = RAMP_float (Angle_Target, RAMP_Angle_Target, 15);
           PID_Control_Smis(Pluck_Motor.Angle_DEG, RAMP_Angle_Target, &Pluck_Place_PIDS ,Pluck_Motor.Speed);
           PID_Control(Pluck_Motor.Speed, Pluck_Place_PIDS.pid_out, &Pluck_Speed_PID);
           limit(Pluck_Speed_PID.pid_out, M2006_LIMIT, -M2006_LIMIT); 
           Can1Send_Shoot[0] = (int16_t)Pluck_Speed_PID.pid_out;       
        } else if(ShootAction == SHOOT_RUNNING){         //连发
            PID_Control(Pluck_Motor.Speed, PLUCK_SPEED, &Pluck_Continue_PID);
            limit(Pluck_Continue_PID.pid_out, M2006_LIMIT, -M2006_LIMIT);
            Can1Send_Shoot[0] = (int16_t)Pluck_Continue_PID.pid_out;
            Angle_Target = Pluck_Motor.Angle_DEG;       //记录角度
            RAMP_Angle_Target = Pluck_Motor.Angle_DEG;
        } else if(ShootAction == SHOOT_STUCKING){       //卡弹退单
            if (Stuck_time == Stuck_thre )
                Stuck_Angle_Target = Pluck_Motor.Angle_DEG - PLUCK_MOTOR_ONE * 2 ;
           Stuck_time++;
           PID_Control_Smis(Pluck_Motor.Angle_DEG, Stuck_Angle_Target, &Pluck_Place_PIDS, Pluck_Motor.Speed);
           PID_Control(Pluck_Motor.Speed, Pluck_Place_PIDS.pid_out, &Pluck_Speed_PID);
           limit(Pluck_Speed_PID.pid_out, M2006_LIMIT, -M2006_LIMIT); 
           Can1Send_Shoot[0] = (int16_t)Pluck_Speed_PID.pid_out;
           if(Stuck_time >= Stuck_thre * 2){
               ShootAction = Last_status;
               Stuck_time = 0;
           }
        }
        if(ShootAction != SHOOT_STUCKING){         //卡弹检测
            if(ShootAction != SHOOT_RUNNING){     //单发不到期望位置的5/4且速度很小时算卡弹
                if ( ( ABS ( Angle_Target - Pluck_Motor.Angle_DEG ) >=  ABS(PLUCK_MOTOR_ONE / 5)  ) &&
                       ABS( Pluck_Motor.Speed ) <= 5 )
                    Stuck_time++;
                else
                    Stuck_time = 0;
            } else {       //连发速度很小时算卡弹
                if (ABS( Pluck_Motor.Speed ) <= 5 )
                    Stuck_time++;
                else
                    Stuck_time = 0;
            }
            if(Stuck_time == Stuck_thre){      // 卡弹时间超过阈值便认为卡弹需退弹
                Last_status = ShootAction;
                ShootAction = SHOOT_STUCKING;
            }
        }
    }
}
