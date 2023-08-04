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

// 舵机X和Y轴的PID控制器
PID_Controller pid_x;
PID_Controller pid_y;
int current_x;
int current_y;

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
//     // pid_init(&pid_x, x, 2.0, 0.0, 0.01); // 目标角度为90度，比例系数为2.0
//     // pid_init(&pid_y, y, 2.0, 0.0, 0.01); // 目标角度为150度，比例系数为2.0
// }
/*
    传入偏移坐标
    ！y在下方为负
    偏移范围 [-120, 120]
    舵机运动角度范围 []
*/
int pwm_x = 640;
int pwm_y = 788;
void servo_test(int x, int y)
{
    // 初始化PID控制器
    if (x == 80)
        pwm_x = pwm_x;
    if (y == 60)
        pwm_y = pwm_y;
    // 舵机往左转
    if (x > 80)
        pwm_x = pwm_x - 5;
    else if (x < 80)
        pwm_x = pwm_x + 5;
    if (y > 60)
        pwm_y = pwm_y + 5;
    else if (y < 60)
        pwm_y = pwm_y - 5;
    if (pwm_x > 900)
        pwm_x = 900;
    else if (pwm_x < 500)
        pwm_x = 500;
    if (pwm_y > 850)
        pwm_y = 850;
    else if (pwm_y < 700)
        pwm_y = 700;
    printf("set_pwm_x:%d, set_pwm_y:%d\r\n", pwm_x, pwm_y);

    setServoPwm(pwm_x, pwm_y);
}

void servo_pid(int x, int y)
{
    if (fabs(x) < 5)
    {
        // Stop control loop, the car has reached the target angle
        return;
    }
    // x方向PID
    //    float target_angle = -10.0; // Target angle for the left turn
    int pwm_x = 640;
    int pwm_y = 788;

    // Initialize PID Controller for the left turn
    pid_init(&pid_x, 0, 2.0, 0, 0);

    float control = pid_control(&pid_x, x);
    pwm_x = pwm_x + control;
    printf("x: %d, control: %.3f, y: %d\r\n", pwm_x, control, pwm_y);
    setServoPwm(pwm_x, 788);
}
void servo_pid_test(int red_x, int red_y, int target_x, int target_y)
{
    int error_x = red_x - target_x;
    int error_y = red_y - target_y;
    int set_pwm_x;
    int set_pwm_y;
    if (fabs(error_x) < 5 && fabs(error_y) < 5)
    {
        return;
    }
    // Initialize PID Controller for the left turn
    pid_init(&pid_x, target_x, 0.66, 0.02, 0.1);
    pid_init(&pid_y, target_y, 0.66, 0.02, 0.1);

    float control_x = pid_control(&pid_x, red_x);
    float control_y = pid_control(&pid_y, red_y);
    control_x = control_x / 10;
    control_y = control_y / 10;
    if (error_x >= 80)
    {
        set_pwm_x = current_x + fabs(control_x);
    }
    else if (error_x < 80)
    {
        set_pwm_x = current_x - fabs(control_x);
    }
    if (error_y >= 60)
    {
        set_pwm_y = current_y - fabs(control_y);
    }
    else if (error_y < 60)
    {
        set_pwm_y = current_y + fabs(control_y);
    }

    if (set_pwm_x > 900)
        set_pwm_x = 900;
    else if (set_pwm_x < 400)
        set_pwm_x = 400;
    if (set_pwm_y > 900)
        set_pwm_y = 900;
    else if (set_pwm_y < 400)
        set_pwm_y = 400;
    printf("control_x:%.3f, control_y:%.3f\r\n", control_x, control_y);
    printf("set_pwm_x:%d, set_pwm_y:%d\r\n", set_pwm_x, set_pwm_y);
    setServoPwm(set_pwm_x, 788);
}
