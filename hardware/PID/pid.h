#ifndef PID_H
#define PID_H

#include "main.h"

#include <stdio.h>
#include "mpu6050.h"
#include <math.h>
#include "usart.h"
#include "delay.h"

void servo_test(int x, int y);
void servo_pid(int x, int y);
void servo_pid_test(int red_x, int red_y, int target_x, int target_y);


#endif
