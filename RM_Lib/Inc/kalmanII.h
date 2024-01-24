/**
 * @file    kalmanII.h
 * @author  yao
 * @date    1-May-2020
 * @brief  二阶卡尔曼滤波器模块
 * @note 此模块依赖DSP库
 * @details
 *   初始化示例
 *                                                  
 *          kalman_filter_t kalman_filter = {
 *              .P_data = {2, 0, 0, 2},               // 协方差矩阵（初始状态估计协方差矩阵）（越可靠收敛越快）
 *              .A_data = {1, 0.001, 0, 1},           // 预测矩阵 （采样时间）（由动态模型确定，模型为直线匀速运动）
 *              .H_data = {2, 0, 0, 1},               // 传感器测量数据矩阵（状态转移矩阵、观测矩阵）（测量值与估计值的关系）
 *              .Q_data = {0.1, 0, 0, 0.1},           // 外部的不确定性（不确定性）（外部噪声协方差矩阵）（模型的不确定性）(由引起噪声的量结合模型确定)
 *              .R_data = {0.1, 0.001, 0.1, 0.001},   // 传感器测量方差（采集数据方差）（传感器测量协方差，二阶，一个位置不确定性，一个速度不确定性）
 *              };
 */


#ifndef _KALMANII_H
#define _KALMANII_H

#include "RMLibHead.h"

RMLIB_CPP_BEGIN

#if !defined(ARM_MATH_CM4) && !defined(ARM_MATH_CM3) && !defined(ARM_MATH_CM7)
#warning "No DSP library support, kalmanII not enable"
#else

#include "arm_math.h"

/**
 * @brief 二阶卡尔曼滤波器
 * @note  恒定速度模型（可对直线匀速运动进行滤波，可预测下一时刻的位置）
 */
typedef struct {
    float raw_value;
    float filtered_value[2];
    float xhat_data[2], xhatminus_data[2], z_data[2], Pminus_data[4], K_data[4];
    float P_data[4];
    float AT_data[4], HT_data[4];
    float A_data[4];
    float H_data[4];
    float Q_data[4];
    float R_data[4];

    /**
     * @brief 滤波器核心
     */
    struct kalman_filtercore {
        float raw_value;
        float filtered_value[2];
        arm_matrix_instance_f32 xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K, E;
    } kalman;
} kalman_filterII_t;

/**
 * @brief 初始化二阶卡尔曼滤波器
 * @param[in] I 二阶卡尔曼滤波器
 */
void kalmanII_Init(kalman_filterII_t *I);

/**
 * @brief 滤波器
 * @param[in] I 二阶卡尔曼滤波器
 * @param[in] signal1 信号1
 * @param[in] signal2 信号2
 * @return 滤波后的信号float数组,长度为2
 */
float *KalmanII_Filter(kalman_filterII_t *I, float signal1, float signal2);

RMLIB_CPP_END

#endif

#endif
