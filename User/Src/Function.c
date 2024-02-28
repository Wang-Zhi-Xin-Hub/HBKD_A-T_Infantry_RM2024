/*!
* @file     Function.c
* @date     2024-1-1
* @brief    全局调用功能函数
*/
#include "Function.h"

/* 设备状态检测函数 */
osStatus_t RM3508_Motor_Temp(RM3508_TypeDef *dst)
{
	if (dst->temp >80)
		return osError;
	else
		return osOK;
}

osStatus_t GM6020_Motor_Temp(GM6020_TypeDef *dst)
{
	if (dst->temp >80)
		return osError;
	else
		return osOK;
}

osStatus_t REMOTE_IfDataError( void )
{
if ( (RC_CtrlData.rc.s1 != 1 && RC_CtrlData.rc.s1 != 3 && RC_CtrlData.rc.s1 != 2)
|| (RC_CtrlData.rc.s2 != 1 && RC_CtrlData.rc.s2 != 3 && RC_CtrlData.rc.s2 != 2)
|| (RC_CtrlData.rc.ch0 > 1684 || RC_CtrlData.rc.ch0 < 364)
|| (RC_CtrlData.rc.ch1 > 1684 || RC_CtrlData.rc.ch1 < 364)
|| (RC_CtrlData.rc.ch2 > 1684 || RC_CtrlData.rc.ch2 < 364)
|| (RC_CtrlData.rc.ch3 > 1684 || RC_CtrlData.rc.ch3 < 364) )
return osError;
else
return osOK;
}

osStatus_t IMU_IfDataError( void )
{
    if(fabs(IMU .EulerAngler .Pitch)>180||fabs (IMU .EulerAngler .Roll)>180||fabs (IMU .EulerAngler .Yaw )>180
        ||(IMU .EulerAngler .Pitch ==0&&IMU .EulerAngler .Roll ==0&&IMU .EulerAngler .Yaw ==0))
        return osError;
    else
        return osOK;
}

/* 斜坡函数（float） */
float RAMP_float( float final, float now, float ramp )
{
    float	buffer =final - now;
    if (buffer > 0)
    {
        if (buffer > ramp)  
                now += ramp;  
        else
                now += buffer;
    }
    else
    {
        if (buffer < -ramp)
                now += -ramp;
        else
                now += buffer;
    }
    return now;
}

/* 电机急停函数 */
void Chassis_Stop()
{
    Communication_Speed_Tx.Chassis_Speed.rotate_ref = 0;
    Communication_Speed_Tx.Chassis_Speed.forward_back_ref = 0;	
    Communication_Speed_Tx.Chassis_Speed.left_right_ref  = 0;
    Communication_Speed_Tx.Shift_flag = 0;
}

void Gimbal_Stop()
{
	PID_Control(Gimbal_Motor[YAW].Speed, 0, &Gimbal_Speed_PID [YAW][Init] );
	PID_Control(Gimbal_Motor[PITCH].Speed, 0, &Gimbal_Speed_PID [PITCH][Init] );
		
	limit(Gimbal_Speed_PID[YAW][Init].pid_out, GM6020_LIMIT, -GM6020_LIMIT);
	limit(Gimbal_Speed_PID[PITCH][Init].pid_out , GM6020_LIMIT, -GM6020_LIMIT);
		
	Can2Send_Gimbal[0] = (int16_t)Gimbal_Speed_PID [YAW][Init].pid_out;
	Can2Send_Gimbal[1] = (int16_t)Gimbal_Speed_PID [PITCH][Init].pid_out;	
}

void Shoot_Stop()
{
    PID_Control(Pluck_Motor.Speed, 0, &Pluck_Speed_PID);
    PID_Control(Shoot_Motor [LEFT].Speed, 0, &Shoot_Speed_PID [LEFT]);
    PID_Control(Shoot_Motor [RIGHT].Speed, 0, &Shoot_Speed_PID [RIGHT]);
    
    limit(Pluck_Speed_PID.pid_out, RM3508_LIMIT, -RM3508_LIMIT);
    limit(Shoot_Speed_PID [LEFT].pid_out, RM3508_LIMIT, -RM3508_LIMIT);
    limit(Shoot_Speed_PID [RIGHT].pid_out, RM3508_LIMIT, -RM3508_LIMIT);
    
    Can1Send_Shoot [0] = (int16_t)Pluck_Speed_PID.pid_out;	
    Can1Send_Shoot [1] = (int16_t)Shoot_Speed_PID [LEFT].pid_out;
    Can1Send_Shoot [2] = (int16_t)Shoot_Speed_PID [RIGHT].pid_out;	    
}

/* 电机关闭函数 */
void Chassis_Close()
{
    Communication_Speed_Tx.Chassis_Speed.rotate_ref = 0;
    Communication_Speed_Tx.Chassis_Speed.forward_back_ref = 0;		
    Communication_Speed_Tx.Chassis_Speed.left_right_ref  = 0;
    Communication_Speed_Tx.Shift_flag = 2; //将Shift标志位用于底盘关闭标志位
    CAN_Send_StdDataFrame(&hcan2, 0x110, (uint8_t *)&Communication_Speed_Tx);
}

void Gimbal_Close()
{
	Can2Send_Gimbal [YAW]=0;
	Can2Send_Gimbal [PITCH]=0;
	MotorSend(&hcan2, 0x1FF, Can2Send_Gimbal);
}

void Shoot_Close()
{
	ShootAction = SHOOT_STOP;
	Can1Send_Shoot[0] = 0;
	Can1Send_Shoot[1] = 0;
	Can1Send_Shoot[2] = 0;
    MotorSend(&hcan1, 0X200, Can1Send_Shoot);
}
