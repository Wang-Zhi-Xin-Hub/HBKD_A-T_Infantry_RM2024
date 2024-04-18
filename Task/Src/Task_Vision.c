#include "Task_Vision.h"

/* 自瞄PC通信 */
Aim_Rx_info Aim_Rx_infopack;
Aim_Rx_t Aim_Rx = { .Fixed_Center_time = 200, .Fixed_Armor_time = 30, .K = 0.01903f};
Aim_Tx_t Aim_Tx;
// k = 0.01903;  // 25°C, 1atm, 小弹丸  空气阻力系数
// k = 0.000556; // 25°C, 1atm, 大弹丸
// k = 0.000530; // 25°C, 1atm, 发光大弹丸

/* 陀螺仪接收解包/发送给视觉任务 */
void Task_IMU_Rx(void *pvParameters)
{
    for(;;)
    {
        if(osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever)){
            IMU_flag ? IMU_Receive(&IMU, Usart2_IMU_Dma[0]) : IMU_Receive(&IMU, Usart2_IMU_Dma[1]);
            Send_to_Vision();   //接收到IMU数据立即发送给视觉
        }
    }
}

/* 给视觉发送信息 */
void Send_to_Vision()
{
    static uint8_t Count = 0, Count_Max = 6; //控制发送频率，根据IMU频率更改，给视觉发送的频率 = IMU频率(1k) / Count_Max
    if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && Aim_Rx.Rx_flag == -1) //Get Vision TimeStamp
        VCOMM_Transmit(0, 1, (uint8_t *)"give_me_time", sizeof("give_me_time"));
    if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && ++Count == Count_Max && Aim_Rx.Rx_flag != -1){ //Send Messgae
        Aim_Tx.TimeStamp     = xTaskGetTickCount() +  Aim_Rx.TimeStamp_setoff;
        Aim_Tx.Quaternions.W = IMU.Quaternions.W;
        Aim_Tx.Quaternions.X = IMU.Quaternions.X;
        Aim_Tx.Quaternions.Y = IMU.Quaternions.Y;
        Aim_Tx.Quaternions.Z = IMU.Quaternions.Z;
        VCOMM_Transmit(1, 1, (uint8_t *)&Aim_Tx, sizeof(Aim_Tx));
        Count = 0;
    }
}

/* 自瞄控制主程序 */
void Aim_Control()
{
    /* 自瞄系统正在测试中 */
    static int16_t Shoot_time_Gap = 500, Shoot_time_cnt = 0;      //射击间隔(击打能量机关需要间隔且单发)
    static uint16_t Predicted_Gap = 0, Bullet_fly_time = 111 , Bullet_Speed = 27;//动态预测时间
    static float horizontal_N, vertical_N, angle_pitch_N, distance_N, Pitch_Angle_Compensation = 0;//弹道模型
    static float distance_min = 0;
    static  int8_t idx = 0;  //索引

      /* 自瞄模式 */
    if ( AimAction != AIM_STOP && PC_State == Device_Online){
        Aim_Rx.Rx_flag ? Predicted_Gap = 0 : Predicted_Gap ++;//量测更新
        
        Aim_Rx.Predicted_Center_time = (Aim_Rx.Fixed_Center_time + Aim_Rx_infopack.delay + Bullet_fly_time + Predicted_Gap) / 1000.0f ;//预测时间ms
        Aim_Rx.Predicted_Armor_time  =  (Aim_Rx.Fixed_Armor_time + Aim_Rx_infopack.delay + Bullet_fly_time + Predicted_Gap) / 1000.0f ;
        
        /* 中心点更新(预测) */
        Aim_Rx.Predicted_PoseN.X = Aim_Rx_infopack.pose.X + Aim_Rx_infopack.pose.Vx * Aim_Rx.Predicted_Center_time;
        Aim_Rx.Predicted_PoseN.Y = Aim_Rx_infopack.pose.Y + Aim_Rx_infopack.pose.Vy * Aim_Rx.Predicted_Center_time;
        Aim_Rx.Predicted_PoseN.Z = Aim_Rx_infopack.pose.Z + Aim_Rx_infopack.pose.Vz * Aim_Rx.Predicted_Center_time;
        
        /* 获得目标装甲板Pose */
        Aim_Rx.Predicted_PoseN.HorizontalDistance = DistanceHorizontal(Aim_Rx.Predicted_PoseN);
        distance_min = Aim_Rx.Predicted_PoseN.HorizontalDistance;
        for (uint8_t i = 0; i < Aim_Rx_infopack.pose.armor_number; i++) {
            float Predicted_theta = Aim_Rx_infopack.pose.theta + Aim_Rx_infopack.pose.omega * Aim_Rx.Predicted_Armor_time + i * (2 * PI / Aim_Rx_infopack.pose.armor_number);
            float r = Aim_Rx_infopack.pose.r1 * ((i+1) % 2) + Aim_Rx_infopack.pose.r2 * (i % 2);    //r1是当前装甲板
            float dz = Aim_Rx_infopack.pose.dz * (i % 2) ; //  另一个装甲板的高度
            Aim_Rx.Predicted_Armor_Pose[i].X = Aim_Rx.Predicted_PoseN.X - r * cos(Predicted_theta);
            Aim_Rx.Predicted_Armor_Pose[i].Y = Aim_Rx.Predicted_PoseN.Y - r * sin(Predicted_theta);
            Aim_Rx.Predicted_Armor_Pose[i].Z = Aim_Rx.Predicted_PoseN.Z + dz;
            Aim_Rx.Predicted_Armor_Pose[i].HorizontalDistance = DistanceHorizontal(Aim_Rx.Predicted_Armor_Pose[i]);
            if(Aim_Rx.Predicted_Armor_Pose[i].HorizontalDistance < distance_min){
                distance_min = Aim_Rx.Predicted_Armor_Pose[i].HorizontalDistance;
                idx = i;
            }
        }
        //2种常见决策方案：
        //1.计算枪管到目标装甲板yaw最小的那个装甲板
        //2.计算距离最近的装甲板           现在是这种
        
        /* 发送预测位置给上位机可视化 */
        Aim_Tx.Tar_Pose.X = Aim_Rx.Predicted_Armor_Pose[idx].X;
        Aim_Tx.Tar_Pose.Y = Aim_Rx.Predicted_Armor_Pose[idx].Y;
        Aim_Tx.Tar_Pose.Z = Aim_Rx.Predicted_Armor_Pose[idx].Z;

        /* 弹道模型更新 */
        if(Aim_Rx.Rx_flag){
        horizontal_N = Aim_Rx.Predicted_Armor_Pose[idx].HorizontalDistance;
        vertical_N   =  Aim_Rx.Predicted_Armor_Pose[idx].Z;
        distance_N = DistanceToOrigin(Aim_Rx.Predicted_Armor_Pose[idx]);
        angle_pitch_N = atan2(vertical_N, horizontal_N);
        Bullet_fly_time = Get_bullet_fly_time(horizontal_N,Bullet_Speed, angle_pitch_N);
        Aim_Rx.PitchOffset = Get_Pitch_Angle_Compensation(horizontal_N, vertical_N, Bullet_Speed);  //单方向空气阻力
//        Aim_Rx.PitchOffset =  ballisticSolver(horizontal_N, vertical_N, Bullet_Speed, st.k);      //完美空气阻力
            Aim_Rx.Rx_flag = 0;
        }

        /* Yaw轴根据目标转速选择跟装甲板还是跟车辆中心 */
        if( ABS(Aim_Rx_infopack.pose.omega) <= 1 )
            Aim_Rx.Predicted_Yaw = atan2( Aim_Rx.Predicted_Armor_Pose[idx].Y, Aim_Rx.Predicted_Armor_Pose[idx].X) * 180 / PI + IMU.EulerAngler.r * 360.0f;
        else
            Aim_Rx.Predicted_Yaw = atan2( Aim_Rx.Predicted_PoseN.Y, Aim_Rx.Predicted_PoseN.X ) * 180 / PI + IMU.EulerAngler.r * 360.0f;
//        Aim_Rx.Predicted_Pitch    = atan2( Aim_Rx.Predicted_PoseN.Z, Aim_Rx.Predicted_PoseN.HorizontalDistance) * 180 / PI;   //中心点
        Aim_Rx.Predicted_Pitch    = atan2( Aim_Rx.Predicted_Armor_Pose[idx].Z, Aim_Rx.Predicted_Armor_Pose[idx].HorizontalDistance) * 180 / PI;//装甲板
        
        /* 得到期望角度（角度 + 弹道补偿）*/
        Aim_Ref.Yaw   =  Aim_Rx.Predicted_Yaw;
        Aim_Ref.Pitch =  - (Aim_Rx.Predicted_Pitch + Aim_Rx.PitchOffset);

        /* 能否开启自瞄(有距离判断  在0.7m - 7m自瞄) */
        if(Aim_Rx_infopack.tracker_status && Aim_Rx.Predicted_PoseN.HorizontalDistance >= 0.7f && Aim_Rx.Predicted_PoseN.HorizontalDistance <= 7.5f){
            Aim_Rx.aim_runing = 1;
            Gyro_Ref.Yaw   = IMU.EulerAngler.ContinuousYaw;
            Mech_Ref.Pitch = Gimbal_Motor[PITCH].Angle;
        } else
            Aim_Rx.aim_runing = 0;

        /* 打弹动态阈值 */
        if( Aim_Rx_infopack.pose.armor_id != 1 || Aim_Rx_infopack.pose.armor_number != 2){
            Aim_Rx.Y_thre = atan2(AM02_ARMOR_X / 2.0f ,Aim_Rx.Predicted_Armor_Pose[idx].HorizontalDistance) * 2.0f/3.0f; 
            Aim_Rx.P_thre = atan2(AM02_ARMOR_Y / 2.0f ,Aim_Rx.Predicted_Armor_Pose[idx].HorizontalDistance) * 2.0f/3.0f;
        } else {
            Aim_Rx.Y_thre = atan2(AM12_ARMOR_X / 2.0f ,Aim_Rx.Predicted_Armor_Pose[idx].HorizontalDistance) * 2.0f/3.0f; 
            Aim_Rx.P_thre = atan2(AM12_ARMOR_Y / 2.0f ,Aim_Rx.Predicted_Armor_Pose[idx].HorizontalDistance) * 2.0f/3.0f;
        }
        
        /* 自瞄启动自动打弹 */
        if( AimAction == AIM_AUTO && Aim_Rx.aim_runing && ShootAction != SHOOT_STOP){
            if( ABS (IMU.EulerAngler.ContinuousYaw - Aim_Ref.Yaw) <= Aim_Rx.Y_thre && ABS (IMU.EulerAngler.Pitch - Aim_Ref.Pitch)<= Aim_Rx.P_thre ){
                    if(Aim_Rx_infopack.pose.armor_number != 5 ){
                        if(ShootAction != SHOOT_STUCKING)
                        ShootAction = SHOOT_RUNNING;
                        } else {                 //打能量机关要单发
                        if(!Shoot_time_cnt--){
                            Shoot_time_cnt = Shoot_time_Gap;
                            if(ShootAction != SHOOT_STUCKING)
                                ShootAction = SHOOT_NORMAL;
                        } else {
                            if(ShootAction != SHOOT_STUCKING)
                                ShootAction = SHOOT_READY;
                        }
                    }
            } else
                if(ShootAction != SHOOT_STUCKING)
                    ShootAction = SHOOT_READY;            
        }
    } else
        Aim_Rx.aim_runing = 0;
}

/* 坐标点到原点水平距离 */
float DistanceHorizontal(Pose_t pose){
    return sqrt(  pose.X * pose.X + pose.Y * pose.Y);
}

/* 坐标点到原点的距离 */
float DistanceToOrigin(Pose_t pose){
    return sqrt(  pose.X * pose.X + pose.Y * pose.Y + pose.Z * pose.Z);
}

/* 获得子弹飞行时间 */
float Get_bullet_fly_time( float horizontal, float bullet_speed, float angle_pitch)
{
    float t;
    t = (float)((exp(Aim_Rx.K * horizontal) - 1) / (Aim_Rx.K * bullet_speed * cos(angle_pitch)));//水平方向求得飞行时间t
    if(t < 0){
        //由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
        //重置t，防止下次调用会出现nan
        t = 0;
        return 0;
    }
    return t;
}
/* 求得Pitch轴角度补偿(单方向空气阻力，适用于远距离，小角度) */
float Get_Pitch_Angle_Compensation(float horizontal, float vertical, float bullet_speed)
{
    float temp_vertical, actual_vertical, error_vertical;
    float pitch, pitch_new;
    
    pitch = atan2(vertical, horizontal);
    temp_vertical = vertical;
    //迭代重力法
    for (uint8_t i = 0; i < 10; i++)
    {
        pitch_new = atan2(temp_vertical, horizontal);
        actual_vertical = monoAirResistance_Model(horizontal, bullet_speed, pitch_new);
        error_vertical = 0.3 * (vertical - actual_vertical);
        temp_vertical = temp_vertical + error_vertical;
        if (fabsf(error_vertical) < 0.001)
            break;
    }
    
    return (pitch_new - pitch) * 180 / PI;
}
/* 单方向空气阻力弹道模型(小弹丸30M/S这个够用了，调调K值就行) */
float monoAirResistance_Model(float horizontal, float bullet_speed, float angle_pitch)
{
    float actual_vertical, t;
    
    t = Get_bullet_fly_time(horizontal, bullet_speed, angle_pitch);
    actual_vertical = bullet_speed * sin(angle_pitch) * t - GRAVITY * t * t / 2; //得出子弹会打到的竖直高度
    
    return actual_vertical;
}

/* 完美弹道模型(大弹丸可能影响大) */
double ballisticSolver(float horizontal, float vertical, double bullet_speed, double k)
{
// FIXME:  NaN Failed 是否因为 vertical 有时候为负
// TODO: 世界坐标系下进行迭代
    double pitch = atan2(vertical, horizontal);
    double temp_vertical = vertical;
    double pitch_new = pitch;

    // 迭代求解
    for (int i = 0; i < 10; ++i) {
        double x = 0.0;
        double y = 0.0;
        double p = tan(pitch_new);
        double v = bullet_speed;
        double u = v / sqrt(1 + p * p);
        double delta_x = horizontal / 10;

        // 使用四阶龙格-库塔法求解微分方程，步长决定精度
        for (int j = 0; j < 10; ++j) {
            double k1_u = -k * u * sqrt(1 + (p*p) );
            double k1_p = -GRAVITY / (u*u);
            double k1_u_sum = u + k1_u * (delta_x / 2);
            double k1_p_sum = p + k1_p * (delta_x / 2);

            double k2_u = -k * k1_u_sum * sqrt(1 + (k1_p_sum*k1_p_sum) );
            double k2_p = -GRAVITY / (k1_u_sum*k1_u_sum);
            double k2_u_sum = u + k2_u * (delta_x / 2);
            double k2_p_sum = p + k2_p * (delta_x / 2);

            double k3_u = -k * k2_u_sum * sqrt(1 + (k2_p_sum*k2_p_sum) );
            double k3_p = -GRAVITY / (k2_u_sum*k2_u_sum);
            double k3_u_sum = u + k3_u * (delta_x / 2);
            double k3_p_sum = p + k3_p * (delta_x / 2);

            double k4_u = -k * k3_u_sum * sqrt(1 + (k3_p_sum*k3_p_sum));
            double k4_p = -GRAVITY / (k3_u_sum*k3_u_sum);

            u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
            p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);

            x += delta_x;
            y += p * delta_x;
        }

        double error = vertical - y;

        // 如果误差满足停止条件，则跳出迭代
        if (fabs(error) <= 0.001) {
            break;
        } else {
            temp_vertical += error;
            pitch_new = atan2(temp_vertical, horizontal);
        }
    }

    // 返回仰角修正值
    return (pitch_new - pitch) * 180 / PI;
}
