#ifndef __KALMAN_H
#define __KALMAN_H
#include "mbed.h"

typedef struct
{
    float dt;      //采样时间
    float angle_f; //角度滤波后
    float angle_m; //角度测量
    float wb_m;    //角速度测量
    float wb_f;    //角速度滤波后
    float q_bias;  //角速度offset
    float P[2][2]; //协方差矩阵
 
    float Q_angle; // Q矩阵
    float Q_gyro;
 
    float R_angle; // R矩阵
} Kalman_pm_st;

void Kalman_Init(Kalman_pm_st* Kalman_pm);

void Kalman_Filter(Kalman_pm_st *kalman_pm);


#endif
