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
int current_angle_x=0;
int current_angle_y=0;
int current_pwm_x = 249;
int current_pwm_y = 249;
void servo_pid(int x ,int y) {
    // 初始化PID控制器
    
	 // 读取当前舵机角度


	pid_init(&pid_x, x, 2.0, 0.0, 0.01); // 目标角度为90度，比例系数为2.0
    pid_init(&pid_y, y, 2.0, 0.0, 0.01); // 目标角度为150度，比例系数为2.0
        // 计算PID控制量并设置舵机位置
        int control_x = pid_control(&pid_x, current_pwm_x);
        int control_y = pid_control(&pid_y, current_pwm_y);
				printf("colx:%d\r\n",control_x);
				printf("coly:%d\r\n",control_y);
	      current_angle_x =current_pwm_x+control_x;
        current_angle_y = current_angle_y+control_y;

				printf("x:%d\r\n",current_angle_x);
				printf("y:%d\r\n",current_angle_y);
	current_angle_x = (current_angle_x - 249)*180/950;
	current_angle_y = (current_angle_y - 249)*300/950;
	      setServoPosition(current_angle_x,current_angle_y);
}


