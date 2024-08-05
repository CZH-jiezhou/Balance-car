#ifndef __ENCODER_H
#define __ENCODER_H
#include "mbed.h"

void E1A_Fall();
void E1A_Rise();
void E1B_Rise();
void E1B_Fall();
void E2A_Fall();
void E2A_Rise();
void E2B_Rise();
void E2B_Fall();
void Encoder_Interruption_Init();
void Obtain_Motor_Speed(long long times, float* motorSpeed_L, float* motorSpeed_R);



#endif
