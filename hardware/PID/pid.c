#include "pid.h"
#include "mpu6050.h"
#include <math.h>
#include "car.h"
#include <stdio.h>
#include "main.h"
#include "xunji.h"
#include "tim.h"


// Define PID Controllers for left turn and straight movement
PID_Controller left_turn_pid;
PID_Controller straight_pid;
// 舵机X和Y轴的PID控制器
PID_Controller pid_x;
PID_Controller pid_y;

GPIO_PinState ledstates[7];

void pid_init(PID_Controller* pid, float target, float Kp, float Ki, float Kd) {
    pid->target = target;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0;
    pid->prev_error = 0.0;
}

float pid_control(PID_Controller* pid, float current_value) {
    float error = pid->target - current_value;
    pid->integral += error;
    float derivative = error - pid->prev_error;
    pid->prev_error = error;
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}

int CarRight90(float target_angle) {
    float current_yaw = 0.0;
//    float target_angle = -75.0; // Target angle for the right turn
    int left_pwm = 500;
    int right_pwm = 500;

    // Initialize PID Controller for the right turn
    pid_init(&left_turn_pid, target_angle, 1.0, 0, 0.01);
    motor_turn_left();
    current_yaw = get_yaw();
    float control = pid_control(&left_turn_pid, current_yaw);
    left_pwm = 500 + control;
    right_pwm = 500 + control;
    car_left(left_pwm, right_pwm);

    if (fabs(current_yaw - target_angle) < 1.5) {
        // Stop control loop, the car has reached the target angle
        return 1;
    }

    return 0;
}

int CarLeft90(float target_angle) {
    float current_yaw = 0.0;
//    float target_angle = -10.0; // Target angle for the left turn
    int left_pwm = 500;
    int right_pwm = 500;

    // Initialize PID Controller for the left turn
    pid_init(&left_turn_pid, target_angle, 1.0, 0, 0.01);

    motor_turn_right();
    current_yaw = get_yaw();
    float control = pid_control(&left_turn_pid, current_yaw);
    left_pwm = 490 - control;
    right_pwm = 490 - control;
    car_right(left_pwm, right_pwm);

    if (fabs(current_yaw - target_angle) < 1.5) {
        // Stop control loop, the car has reached the target angle
        return 1;
    }

    return 0;
}
void Cargo(float target_angle) {
    float current_yaw = 0.0;
    uint16_t left_pwm = 100;
    uint16_t right_pwm = 100;

    // Initialize PID Controller for straight movement
    pid_init(&straight_pid, target_angle, 1.0, 0, 0.01);

    current_yaw = get_yaw();
    float control = pid_control(&straight_pid, current_yaw);
    left_pwm = 100 - control;
    right_pwm = 100 + control;   
     car_stright(left_pwm, right_pwm);      
    if (fabs(current_yaw - target_angle) < 1.5) {
        // Stop control loop, the car has reached the target angle
        return;
    }
}
void CarStraight(float target_angle) {
    float current_yaw = 0.0;
    uint16_t left_pwm = 500;
    uint16_t right_pwm = 500;

    // Initialize PID Controller for straight movement
    pid_init(&straight_pid, target_angle, 5.0, 0, 0.01);

    current_yaw = get_yaw();
    float control = pid_control(&straight_pid, current_yaw);
    left_pwm = 500 - control;
    right_pwm = 500 + control;

    int flag = readLEDsState(ledstates);
    if (flag) {
        if (flag == 1)
            car_stright(left_pwm, right_pwm);
        else if (flag == 2) {
            car_stright(left_pwm * 0.6, right_pwm);
        } else if (flag == 3) {
            car_stright(left_pwm, right_pwm * 0.6);
        }
    }

    if (fabs(current_yaw - target_angle) < 1.5) {
        // Stop control loop, the car has reached the target angle
        return;
    }
}


//舵机pid
PID pid;

void PID_Init()
{
	pid.X_Kp = 0.75;
	pid.X_Ki = 0.003;
	pid.X_Kd = 0.01;
	pid.X_err = 0;
	pid.X_err_sum = 0;
	pid.X_err_last = 0;
	
	pid.Y_Kp=0.66;
	pid.Y_Ki=0.005;
	pid.Y_Kd=0.01;
	pid.Y_err=0;
	pid.Y_err_sum=0;
	pid.Y_err_last=0;
}

//水平方向
//int lim=18;
int lim=27;
int PID_Level(int receivedX)
{
	int out;
	
	pid.X_err = receivedX;
	pid.X_err_sum += pid.X_err;
	out = pid.X_Kp*(pid.X_err)
		+ pid.X_Ki*(pid.X_err_sum)
		+ pid.X_Kd*(pid.X_err - pid.X_err_last);
	pid.X_err_last = pid.X_err;
	
	if (out>=lim)
		return lim;
	else if (out<=-lim+6)
		return -lim;
	if (fabs(receivedX)<=10)
		return 0;
	return out;
}

//垂直方向
int PID_vertical(int receivedY)
{
	int out;
		
	pid.Y_err = -receivedY;
	pid.Y_err_sum += pid.Y_err;
	out = pid.Y_Kp*(pid.Y_err)
		+ pid.Y_Ki*(pid.Y_err_sum)
		+ pid.Y_Kd*(pid.Y_err - pid.Y_err_last);
	pid.Y_err_last = pid.Y_err;
	if (out>=lim)
		return lim;
	else if (out<=-lim+6)
		return -lim;
	if (fabs(receivedY)<=10)
		return 0;
	return out;
}


