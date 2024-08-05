#include "motor.h"

PwmOut a1(A1);
PwmOut a2(A2);
BufferedSerial serial_port(D5, D4, 9600);
DigitalOut BIN2(A0);
DigitalOut BIN1(D13);
DigitalOut AIN2(D2);
DigitalOut AIN1(D6);

int received_value_1 = 0;  // Variable to store received value
int received_value_2 = 0;  // Variable to store received value
int received_value_3 = 0;  // Variable to store received value
int received_value_4 = 0;  // Variable to store received value
int received_value_5 = 0;  // Variable to store received value
int Max = 0;

//直立环
float Kp_stand = 0.0;
float Kd_stand = 0.0;
float err_stand = 0.0;
float goal = 0;
//速度环
float Kp_speed = 0.0;
float Ki_speed = 0.0;
float Speed_sum = 0.0;
//电机死区
float died_area = 0;


void Motor_run(float speed)
{
    
    a1.period_ms(1);
    a2.period_ms(1);

    if(speed < 0)
    {
        BIN2 = 1;
        BIN1 = 0;
        AIN2 = 1;
        AIN1 = 0;

        a1.write(-(speed+died_area));
        a2.write(-(speed+died_area));
    }
    else 
    {
        BIN2 = 0;
        BIN1 = 1;
        AIN2 = 0;
        AIN1 = 1;
        a1.write(speed-died_area);
        a2.write(speed-died_area);
    }

}

void read_serial()
{
    char buffer[3] = {'\0'};
    if (serial_port.readable())
    {
        thread_sleep_for(50);
        serial_port.read(&buffer[0], 3);
        received_value_1 = atoi(buffer);
        thread_sleep_for(50);
        serial_port.read(&buffer[0], 3);
        received_value_2 = atoi(buffer);
        thread_sleep_for(50);
        serial_port.read(&buffer[0], 3);
        received_value_3 = atoi(buffer);
        thread_sleep_for(50);
        serial_port.read(&buffer[0], 3);
        received_value_4 = atoi(buffer);
        thread_sleep_for(50);
        serial_port.read(&buffer[0], 3);
        received_value_5 = atoi(buffer);

        Kp_stand = received_value_1/100.0;
        Kd_stand = received_value_2/1000.0;
        Kp_speed = -received_value_3/1000.0;
        Ki_speed = Kp_speed/200.0;
        goal = -received_value_4/100.0;
        Max = received_value_5;


    }
}

float PID_Stand_Control(float measure_angle,float speed)
{
    read_serial();
    err_stand = measure_angle - goal;
    return Kp_stand*err_stand + Kd_stand*speed;
}

float PID_Speed_Control(float speed)
{
    Speed_sum += speed;
    if(Speed_sum>=Max)
    {
        Speed_sum = Max;
    }
    else if(Speed_sum <= -Max)
    {
        Speed_sum = -Max;
    }
    return Kp_speed*speed + Ki_speed*Speed_sum;
}