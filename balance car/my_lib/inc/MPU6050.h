#ifndef __MPU6050_H
#define __MPU6050_H
#include "mbed.h"

#define I2CSDA D0
#define I2CSCL D1
#define MPU6050_add_write 0xD0 //7位地址码+读位：0x68<<1 | 0xD0 读地址
#define MPU6050_add_read 0xD1  //7位地址码+写位：0x68<<1 | 0xD1 写地址
#define Configuration_add 0x1a
#define Power_Configuration_add1 0x6b//电源配置寄存器1的地址
#define Power_Configuration_add2 0x6c//电源配置寄存器2的地址
#define Gyroscope_Configuration_add 0x1b
#define Accelerometer_Configuration_add 0x1c
#define Sampling_rate_division_add 0x19//采样率分频寄存器地址
#define ACCEL_XOUT_H 0x3b
//#define ACCEL_XOUT_L 0x3c
#define ACCEL_YOUT_H 0x3D
//#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
//#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define Gyro_XOUT_H 0x43
//#define Gyro_XOUT_L 0x44
#define Gyro_YOUT_H 0x45
//#define Gyro_YOUT_L 0x46
#define Gyro_ZOUT_H 0x47
//#define Gyro_ZOUT_L 0x48

void MPU6050_init();

void MPU_6050_Getdata(int16_t *ACC_X, int16_t *ACC_Y, int16_t *ACC_Z,
                      int16_t *Gyro_X, int16_t *Gyro_Y, int16_t *Gyro_Z, int16_t *Temp);

void Clibration(int16_t *ACC_X, int16_t *ACC_Y, int16_t *ACC_Z,
                int16_t *Gyro_X, int16_t *Gyro_Y, int16_t *Gyro_Z,
                float *arry1, float *arry2, float *arry_average_angle);

void Get_Euler_angles(float a_x, float a_y, float a_z, float* roll, float* pitch);

void Get_Euler_angles_Gyro(float dt, float* integralX, float* integralY, float* integralZ, float X_rad_s, float Y_rad_s, float Z_rad_s);

void Get_Angular_Velocity(int16_t Gyro_X, int16_t Gyro_Y, int16_t Gyro_Z, float* clibration_arry_Gyro, float* X_rad_s, float* Y_rad_s, float* Z_rad_s);


#endif
