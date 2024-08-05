#include "mbed.h"
#include "MPU6050.h"
#include "Kalman.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include "math.h"
#include "encoder.h"
#include "motor.h"
#include "chrono" // Include the chrono library for timing functions


using namespace std::chrono;


Timer t; // Create a Timer object 't' for keeping track of time


int main()

{



    int16_t ACC_X, ACC_Y, ACC_Z, Gyro_X, Gyro_Y, Gyro_Z, Temp;
    float X_g, Y_g, Z_g, X_rad_s, Y_rad_s, Z_rad_s,roll,pitch,final_Pitch,final_Roll;
    float clibration_arry_Gyro[3] = {0,0,0};
    float clibration_arry_Acc[3] = {0,0,0};
    float clibration_arry_angle[2];
    float dt = 0;
    float speed_L = 0;
    float speed_R = 0;
    float average_speed = 0;
    float Pitch_Speed = 0;
    float PID_Out_Put = 0;
    static const int integral_interval = 10;//10ms
    auto integral_tick1 = 0; // Variable to store the time of the last event for key1
    static float integralX = 0.0f, integralY = 0.0f, integralZ = 0.0f ,integralX_revise = 0.0f, integralY_revise, integralZ_revise;
    Kalman_pm_st  Kalman_pm;   //定义结构体
    
    Kalman_Init(&Kalman_pm);    //给结构体赋初值和修改参数
    Encoder_Interruption_Init();
    MPU6050_init();
    Clibration(&ACC_X, &ACC_Y, &ACC_Z, &Gyro_X, &Gyro_Y, &Gyro_Z,clibration_arry_Gyro,clibration_arry_Acc,clibration_arry_angle);
    t.start(); // Start the timer

    while(1)
    {
        auto elapsed_time = t.elapsed_time(); // Get the elapsed time
        auto times = chrono::duration_cast<chrono::milliseconds>(elapsed_time).count(); // 创建计数器，单位ms

        MPU_6050_Getdata(&ACC_X, &ACC_Y, &ACC_Z, &Gyro_X, &Gyro_Y, &Gyro_Z, &Temp);

        X_g = ((ACC_X-292)/32768.0)*2/0.9925 - 0.007;
        Y_g = ((ACC_Y/1.023-162)/32768.0)*2/0.985 + 0.015;
        Z_g = ((ACC_Z *0.9889-65)/32768.0)*2;

        Get_Euler_angles(X_g, Y_g, Z_g, &roll, &pitch);

        // roll = roll - clibration_arry_angle[0];
        // pitch = pitch - clibration_arry_angle[1];

        Get_Angular_Velocity(Gyro_X, Gyro_Y, Gyro_Z, clibration_arry_Gyro, &X_rad_s, &Y_rad_s, &Z_rad_s);


        dt = (times - integral_tick1)/1000.0;
        integral_tick1 = times; // Update the time of the last event for key1
        Kalman_pm.dt = dt;

        
        /*参数输入*/
        Kalman_pm.angle_m = pitch;   //加速度计求解角度的值
        Kalman_pm.wb_m = -Y_rad_s;      //陀螺仪的角速度

        Kalman_Filter(&Kalman_pm);   //调用卡尔曼滤波
            
        //Get_Euler_angles_Gyro(dt, &integralX, &integralY, &integralZ, X_rad_s, Y_rad_s, Z_rad_s);
        final_Pitch = Kalman_pm.angle_f;   //滤波后的角度
        Pitch_Speed = Kalman_pm.wb_f;          //滤波后的角速度


        Obtain_Motor_Speed(times,&speed_L,&speed_R);
        average_speed = speed_R;


        
        
        //printf("%f\r\n",0.1);
        //Motor_run(-0.1);
        Motor_run(PID_Stand_Control(final_Pitch,Pitch_Speed)/90.0 + PID_Speed_Control(average_speed));

        //test_pid();
        //printf("final_Pitch: %f,  Pitch_Speed: %f\r\n",final_Pitch,Pitch_Speed); 
        //printf("speed_R: %f,  speed_L: %f\r\n",speed_R,speed_L);
        //printf("Pitch: %f,  Roll: %f\r\n",pitch,roll); 
        //printf("pid_out = %f\r\n",PID_Speed_Control(average_speed));
    }
    return 0;
}
