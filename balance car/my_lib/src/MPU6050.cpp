#include "MPU6050.h"
#include <cstdint>
#include "chrono" // Include the chrono library for timing functions



I2C MPU6050(I2CSDA,I2CSCL);

char ConfigRegister_MPU6050[2];
char ConfigRegister_Gyroscope[2];
char ConfigRegister_Accelerometer[2];
char ConfigRegister_Power1[2];
char ConfigRegister_Power2[2];
char ConfigRegister_Sampling_rate[2];

char add_test[1];



void MPU6050_init()
{
    //写入电源配置寄存器1，解除睡眠模式并配置时钟源
    ConfigRegister_Power1[0] = Power_Configuration_add1;
    ConfigRegister_Power1[1] = 0x01;//解除睡眠模式并选择陀螺仪时钟
    MPU6050.write(MPU6050_add_write,ConfigRegister_Power1,2);

    wait_us(1000);

    //写入电源配置寄存器2，
    ConfigRegister_Power2[0] = Power_Configuration_add2;
    ConfigRegister_Power2[1] = 0x00;
    MPU6050.write(MPU6050_add_write,ConfigRegister_Power2,2);
/*
    wait_us(1000);
    取消注释配置为10分频，保留注释则默认不分频
    ConfigRegister_Sampling_rate[0] = Sampling_rate_division_add;
    ConfigRegister_Sampling_rate[1] = 0x09;
    MPU6050.write(MPU6050_add_write,ConfigRegister_Sampling_rate,2);

    wait_us(1000);
*/
    //配置总寄存器
    ConfigRegister_MPU6050[0] = Configuration_add;
    ConfigRegister_MPU6050[1] = 0x06;//设置为最平滑滤波
    MPU6050.write(MPU6050_add_write,ConfigRegister_MPU6050,2);

    wait_us(1000);
    //配置陀螺仪寄存器
    ConfigRegister_Gyroscope[0] = Gyroscope_Configuration_add;
    ConfigRegister_Gyroscope[1] = 0x18;//量程设置为+-2000°/s
    MPU6050.write(MPU6050_add_write,ConfigRegister_Gyroscope,2);

    wait_us(1000);
    //配置加速度寄存器
    ConfigRegister_Accelerometer[0] = Accelerometer_Configuration_add;
    ConfigRegister_Accelerometer[1] = 0x00;//量程设置为+-2g
    MPU6050.write(MPU6050_add_write,ConfigRegister_Accelerometer,2);

    ConfigRegister_MPU6050[0] = 0x75;
    MPU6050.write(MPU6050_add_write ,ConfigRegister_MPU6050,1);
    MPU6050.read(MPU6050_add_read,add_test,1);

    if(add_test[0] == 0x68)
    {
        printf("successfully connect to MPU6050\r\n");
    }

    else
    {
        printf("failed to connect with MPU6050\r\n");
    }
    
}

void MPU_6050_Getdata(int16_t *ACC_X, int16_t *ACC_Y, int16_t *ACC_Z,
                      int16_t *Gyro_X, int16_t *Gyro_Y, int16_t *Gyro_Z, int16_t *Temp)
{
    char read_register[1];
    char data_read[2];
    int16_t M, L;

    read_register[0] = ACCEL_XOUT_H;
    MPU6050.write(MPU6050_add_write ,read_register,1);
    MPU6050.read(MPU6050_add_read,data_read,2);
    M = data_read[0]<<8;
    L  =data_read[1];
    *ACC_X = M | L;

    read_register[0] = ACCEL_YOUT_H;
    MPU6050.write(MPU6050_add_write ,read_register,1);
    MPU6050.read(MPU6050_add_read,data_read,2);
    M = data_read[0]<<8;
    L  =data_read[1];
    *ACC_Y = M | L;

    read_register[0] = ACCEL_ZOUT_H;
    MPU6050.write(MPU6050_add_write ,read_register,1);
    MPU6050.read(MPU6050_add_read,data_read,2);
    M = data_read[0]<<8;
    L  =data_read[1];
    *ACC_Z = M | L;

    read_register[0] = Gyro_XOUT_H;
    MPU6050.write(MPU6050_add_write ,read_register,1);
    MPU6050.read(MPU6050_add_read,data_read,2);
    M = data_read[0]<<8;
    L  =data_read[1];
    *Gyro_X = M | L;

    read_register[0] = Gyro_YOUT_H;
    MPU6050.write(MPU6050_add_write ,read_register,1);
    MPU6050.read(MPU6050_add_read,data_read,2);
    M = data_read[0]<<8;
    L  =data_read[1];
    *Gyro_Y = M | L;

    read_register[0] = Gyro_ZOUT_H;
    MPU6050.write(MPU6050_add_write ,read_register,1);
    MPU6050.read(MPU6050_add_read,data_read,2);
    M = data_read[0]<<8;
    L  =data_read[1];
    *Gyro_Z = M | L; 

    read_register[0] = TEMP_OUT_H;
    MPU6050.write(MPU6050_add_write ,read_register,1);
    MPU6050.read(MPU6050_add_read,data_read,2);
    M = data_read[0]<<8;
    L  =data_read[1];
    *Temp = M | L; 
}

void Get_Euler_angles(float a_x, float a_y, float a_z, float* roll, float* pitch)
{

    *roll = atan(a_y/a_z)/3.1415926*180;
    float c = sqrt(a_y * a_y + a_z * a_z);
    *pitch = -atan(a_x/c)/3.1415926*180;
}

void Clibration(int16_t *ACC_X, int16_t *ACC_Y, int16_t *ACC_Z,
                int16_t *Gyro_X, int16_t *Gyro_Y, int16_t *Gyro_Z,
                float *arry1, float *arry2, float *arry_average_angle) 
{

    int16_t n, temp;
    float X_g, Y_g, Z_g, roll, pitch;
    roll = 0;
    pitch = 0;
    arry_average_angle[0] = 0;
    arry_average_angle[1] = 0;
    arry1[0] = 0;
    arry1[1] = 0;
    arry1[2] = 0;
    arry2[0] = 0;
    arry2[1] = 0;
    arry2[2] = 0;


    printf("三秒后开始校准，请让传感器平放在桌面，直到校准结束\r\n");
    wait_us(3000000);
    printf("校准开始\r\n");

    for(n = 0;n<1000;n++)
    {
        
        MPU_6050_Getdata(ACC_X, ACC_Y, ACC_Z, Gyro_X, Gyro_Y, Gyro_Z, &temp);
        
        arry1[0] = arry1[0] + *Gyro_X;
        arry1[1] = arry1[1] + *Gyro_Y;
        arry1[2] = arry1[2] + *Gyro_Z;

        arry2[0] = arry2[0] + *ACC_X;
        arry2[1] = arry2[1] + *ACC_Y;
        arry2[2] = arry2[2] + *ACC_Z;

        wait_us(1000);
    }

    n = 0;

    for(n = 0;n<1000;n++)
    {
        MPU_6050_Getdata(ACC_X, ACC_Y, ACC_Z, Gyro_X, Gyro_Y, Gyro_Z, &temp);
        X_g = ((*ACC_X-292)/32768.0)*2/0.9925 - 0.007;
        Y_g = ((*ACC_Y/1.023-162)/32768.0)*2/0.985 + 0.015;
        Z_g = ((*ACC_Z *0.9889-65)/32768.0)*2;
        Get_Euler_angles(X_g,Y_g,Z_g,&roll,&pitch);
        arry_average_angle[0] += roll;
        arry_average_angle[1] += pitch;
    }


    arry1[0] = arry1[0]/1000;
    arry1[1] = arry1[1]/1000;
    arry1[2] = arry1[2]/1000;
    arry2[0] = arry2[0]/1000;
    arry2[1] = arry2[1]/1000;
    arry2[2] = arry2[2]/1000;
    arry_average_angle[0] = arry_average_angle[0]/1000.0;
    arry_average_angle[1] = arry_average_angle[1]/1000.0;


    printf("校准结束\r\n");

}

void Get_Euler_angles_Gyro(float dt, float* integralX, float* integralY, float* integralZ, float X_rad_s, float Y_rad_s, float Z_rad_s)
{
        *integralX += X_rad_s * dt;  //d(Roll)
        *integralY += Y_rad_s * dt;  //d(Pitch)
        *integralZ += Z_rad_s * dt;  //d(Yaw)
        // 360°一个循环
        if (*integralX > 360)
            *integralX -= 360;
        if (*integralX < -360)
            *integralX += 360;
        if (*integralY > 360)
            *integralY -= 360;
        if (*integralY < -360)
            *integralY += 360;
    
        if (*integralZ > 360)
            *integralZ -= 360;
        if (*integralZ < -360)
            *integralZ += 360;
}

void Get_Angular_Velocity(int16_t Gyro_X, int16_t Gyro_Y, int16_t Gyro_Z, float* clibration_arry_Gyro, float* X_rad_s, float* Y_rad_s, float* Z_rad_s)
{
    *X_rad_s = ((Gyro_X - clibration_arry_Gyro[0])/32768.0)*2000;
    *Y_rad_s = ((Gyro_Y - clibration_arry_Gyro[1])/32768.0)*2000;
    *Z_rad_s = ((Gyro_Z - clibration_arry_Gyro[2])/32768.0)*2000;

    if(*X_rad_s <= 0.07 && *X_rad_s >= -0.07)
        {
            *X_rad_s = 0;
        }

        if(*Y_rad_s <= 0.07 && *Y_rad_s >= -0.07)
        {
            *Y_rad_s = 0;
        }

        if(*Z_rad_s <= 0.07 && *Z_rad_s >= -0.07)
        {
            *Z_rad_s = 0;
        }
}
