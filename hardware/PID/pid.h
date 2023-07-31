#ifndef PID_H
#define PID_H

#include "main.h"

#include <stdio.h>
#include "mpu6050.h"
#include <math.h>
#include "usart.h"
#include "delay.h"

extern int CarLeft90(float target_angle);
extern int CarRight90(float target_angle);
extern void CarStraight(float target_angle);
extern void CarBack(void);
void servo_pid(int  x,int  y) ;
void Cargo(float target_angle);
// Define PID Controller structure
typedef struct {
    float target;
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
} PID_Controller;

void pid_init(PID_Controller* pid, float target, float Kp, float Ki, float Kd);
float pid_control(PID_Controller* pid, float current_value);


//typedef struct 
//{
//	float X_Kp;
//	float X_Ki;
//	float X_Kd;
//	float X_err;
//	float X_err_sum;
//	float X_err_last;	
//	
//	float Y_Kp;
//	float Y_Ki;
//	float Y_Kd;
//	float Y_err;
//	float Y_err_sum;
//	float Y_err_last;	
//}PID;

//void PID_Init(void);
//int PID_Level(int x);
//int PID_vertical(int y);
#endif
