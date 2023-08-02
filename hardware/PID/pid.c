#include "pid.h"
#include "mpu6050.h"
#include <math.h>
#include "car.h"
#include <stdio.h>
#include "main.h"
#include "xunji.h"
#include "tim.h"
// Define PID Controller structure
typedef struct
{
    float target;
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
} PID_Controller;

// ���X��Y���PID������
PID_Controller pid_x;
PID_Controller pid_y;

void pid_init(PID_Controller *pid, float target, float Kp, float Ki, float Kd)
{
    pid->target = target;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0;
    pid->prev_error = 0.0;
}

float pid_control(PID_Controller *pid, float current_value)
{
    float error = pid->target - current_value;
    pid->integral += error;
    float derivative = error - pid->prev_error;
    pid->prev_error = error;
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}

// void ServoPidInit()
// {
//     // pid_init(&pid_x, x, 2.0, 0.0, 0.01); // Ŀ��Ƕ�Ϊ90�ȣ�����ϵ��Ϊ2.0
//     // pid_init(&pid_y, y, 2.0, 0.0, 0.01); // Ŀ��Ƕ�Ϊ150�ȣ�����ϵ��Ϊ2.0
// }
/*
    ����ƫ������
    ��y���·�Ϊ��
    ƫ�Ʒ�Χ [-120, 120]
    ����˶��Ƕȷ�Χ []
*/
int pwm_x = 50;
int pwm_y = 50;
void servo_test(int x, int y)
{
    // ��ʼ��PID������

    // �������ת
    if (x > 10)
        pwm_x = pwm_x - 50;
    else if (x < -10)
        pwm_x = pwm_x + 50;
    if (y > 10)
        pwm_y = pwm_y + 50;
    else if (y < -10)
        pwm_y = pwm_y - 50;
    if (pwm_x > 1199)
        pwm_x = 1199;
    else if (pwm_x < 299)
        pwm_x = 299;
    if (pwm_y > 1199)
        pwm_y = 1199;
    else if (pwm_y < 299)
        pwm_y = 299;
    setServoPwm(pwm_x, pwm_y);
}

void servo_pid(int x, int y)
{
    if (fabs(x) < 5)
    {
        // Stop control loop, the car has reached the target angle
        return;
    }
    // x����PID
    //    float target_angle = -10.0; // Target angle for the left turn
    int pwm_x = 500;
    int pwm_y = 500;

    // Initialize PID Controller for the left turn
    pid_init(&pid_x, 0, 2.0, 0, 0);

    float control = pid_control(&pid_x, x);
    pwm_x = pwm_x + control;
    printf("x: %d, control: %.3f, y: %d\r\n", pwm_x, control, pwm_y);
    setServoPwm(pwm_x, 500);
}
