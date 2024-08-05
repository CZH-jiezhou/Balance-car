#include "encoder.h"

InterruptIn E1A(A6, PullDown);
InterruptIn E1B(A5, PullDown);
InterruptIn E2A(D3, PullDown);
InterruptIn E2B(A3, PullDown);

volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
float motorSpeed1 = 0;
float motorSpeed2 = 0;
auto Current_Time1 = 0;
auto Current_Time2 = 0;
const int dt = 20; //10ms
// Number of pulses per revolution for the motor
const int pulsesPerRevolution = 1059;
int rotation_dir1 = 0;
int rotation_dir2 = 0;


void E1A_Fall()
{
    pulseCount1++;  // Increment the pulse count on each encoder signal
    if(E1B == 1)
    {
        rotation_dir1 = 1;
    }
    else if(E1B == 0)
    {
        rotation_dir1 = 0;
    }
}

void E1A_Rise()
{
    pulseCount1++;  // Increment the pulse count on each encoder signal
    if(E1B == 0)
    {
        rotation_dir1 = 1;
    }
    else if(E1B == 1)
    {
        rotation_dir1 = 0;
    }
}

void E1B_Rise()
{
    pulseCount1++;  // Increment the pulse count on each encoder signal
    if(E1A == 1)
    {
        rotation_dir1 = 1;
    }
    else if(E1A == 0)
    {
        rotation_dir1 = 0;
    }
}

void E1B_Fall()
{
    pulseCount1++;  // Increment the pulse count on each encoder signal
    if(E1A == 0)
    {
        rotation_dir1 = 1;
    }
    else if(E1A == 1)
    {
        rotation_dir1 = 0;
    }
}

void E2A_Fall()
{
    pulseCount2++;  // Increment the pulse count on each encoder signal
    if(E2B == 1)
    {
        rotation_dir2 = 1;
    }
    else if(E2B == 0)
    {
        rotation_dir2 = 0;
    }
}

void E2A_Rise()
{
    pulseCount2++;  // Increment the pulse count on each encoder signal
    if(E2B == 0)
    {
        rotation_dir2 = 1;
    }
    else if(E2B == 1)
    {
        rotation_dir2 = 0;
    }
}

void E2B_Rise()
{
    pulseCount2++;  // Increment the pulse count on each encoder signal
    if(E2A == 1)
    {
        rotation_dir2 = 1;
    }
    else if(E2A == 0)
    {
        rotation_dir2 = 0;
    }
}

void E2B_Fall()
{
    pulseCount2++;  // Increment the pulse count on each encoder signal
    if(E2A == 0)
    {
        rotation_dir2 = 1;
    }
    else if(E2A == 1)
    {
        rotation_dir2 = 0;
    }
}

void Encoder_Interruption_Init()
{
    E1A.rise(&E1A_Rise);
    E1A.fall(&E1A_Fall);
    E1B.rise(&E1B_Rise);
    E1B.fall(&E1B_Fall);
    E2A.rise(&E2A_Rise);
    E2A.fall(&E2A_Fall);
    E2B.rise(&E2B_Rise);
    E2B.fall(&E2B_Fall);
}

void Obtain_Motor_Speed(long long times, float* motorSpeed_L, float* motorSpeed_R)
{
    if(times - Current_Time1 >= dt) // Check if the debounce interval has passed
            {
                Current_Time1 = times; // Update the time of the last event for key1
                // Calculate motor speed in revolutions per minute (RPM)
                if(rotation_dir1)
                {
                    motorSpeed1 = (float)pulseCount1 * 1000 / (pulsesPerRevolution * dt); //计算角速度 单位圈每毫秒
                }
                else 
                {
                    motorSpeed1 = -(float)pulseCount1 * 1000 / (pulsesPerRevolution * dt); //计算角速度 单位圈每毫秒
                }
                // Reset pulse count and timer for the next measurement
                pulseCount1 = 0;
            }

    if(times - Current_Time2 >= dt) // Check if the debounce interval has passed
            {
                Current_Time2 = times; // Update the time of the last event for key1
                // Calculate motor speed in revolutions per minute (RPM)
                if(rotation_dir2 == 0)
                {
                    motorSpeed2 = (float)pulseCount2 * 1000 / (pulsesPerRevolution * dt); //计算角速度 单位圈每毫秒
                }
                else 
                {
                    motorSpeed2 = -(float)pulseCount2 * 1000 / (pulsesPerRevolution * dt); //计算角速度 单位圈每毫秒
                }
                // Reset pulse count and timer for the next measurement
                pulseCount2 = 0;
            }
    // Print the motor speed to the console
    * motorSpeed_L = motorSpeed1;
    * motorSpeed_R = motorSpeed2;
    
}