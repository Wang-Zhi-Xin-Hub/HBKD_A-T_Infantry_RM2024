/**
 * @file    PID.c
 * @author  yao
 * @date    1-May-2020
 * @brief   PID模块
 */

#include "PID.h"

void PID_Control(float current, float expected, PID *parameter) {
    parameter->error_last = parameter->error_now;
    parameter->error_now = expected - current;

    if(fabs(parameter->error_now) < parameter->DeadBand)
        parameter->error_now = 0.0f;
        if(fabs(parameter->error_now) < parameter->error_thre)
        {
            if(parameter->error_now <= 0)
                parameter->error_inter += (parameter->error_now + parameter->DeadBand);
            else
                parameter->error_inter += (parameter->error_now - parameter->DeadBand);
        }
        
        limit(parameter->error_inter, parameter->limit, -parameter->limit);
        
        parameter->pid_out = parameter->Kp * parameter->error_now + parameter->Ki * parameter->error_inter +
                             parameter->Kd * (parameter->error_now - parameter->error_last);
}

void PID_Control_Smis(float current, float expected, PID_Smis *parameter, float speed) {
    parameter->error_now = expected - current;

    if(fabs(parameter->error_now) < parameter->DeadBand)
        parameter->error_now = 0.0f;
    
        if(fabs(parameter->error_now) < parameter->error_thre)
        {
            if(parameter->error_now <= 0)
                parameter->error_inter += (parameter->error_now + parameter->DeadBand);
            else
                parameter->error_inter += (parameter->error_now - parameter->DeadBand);
        }

        limit(parameter->error_inter, parameter->limit, -parameter->limit);

        parameter->pid_out = parameter->Kp * parameter->error_now + parameter->Ki * parameter->error_inter +
                             parameter->Kd * speed;
}

float PID_Increment(float current, float expect, PID_ADD *parameter) {
    parameter->error_now = expect - current;

    parameter->increament =
            parameter->Kp * (parameter->error_now - parameter->error_next) + parameter->Ki * (parameter->error_now) +
            parameter->Kd * (parameter->error_now - 2 * parameter->error_next + parameter->error_last);

    parameter->error_last = parameter->error_next;
    parameter->error_next = parameter->error_now;

    return parameter->increament;
}

float FeedForward_Calc(FeedForward_Typedef *FF){
    
    FF->Out = FF->Now_DeltIn*FF->K1 + (FF->Now_DeltIn - FF->Last_DeltIn)*FF->K2;
    FF->Last_DeltIn = FF->Now_DeltIn;
    
    limit(FF->Out,FF->OutMax,-FF->OutMax);

    return FF->Out;
}