#ifndef __MOTOR_H
#define __MOTOR_H
#include "mbed.h"

void Motor_run(float speed);
float PID_Stand_Control(float measure_angle,float speed);
float PID_Speed_Control(float speed);
void read_serial();
//void test_pid();
#endif
