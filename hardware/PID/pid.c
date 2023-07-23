#include "pid.h"
#include "mpu6050.h"
#include <math.h>
#include "car.h"
#include <stdio.h>
#include "main.h"
#include "xunji.h"

// Define PID Controller structure
typedef struct {
    float target;
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
} PID_Controller;

// Define PID Controllers for left turn and straight movement
PID_Controller left_turn_pid;
PID_Controller straight_pid;

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
    left_pwm = 490 + control;
    right_pwm = 490 + control;
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
        motor_forward();
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

