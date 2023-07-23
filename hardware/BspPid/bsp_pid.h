#ifndef __BSP_PID_H
#define __BSP_PID_H

#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct
{
    float target_val; // 目标值
    float actual_val; // 实际值
    float err;        // 定义偏差值
    float err_last;   // 定义上一个偏差值
    float Kp, Ki, Kd; // 定义比例、积分、微分系数
    float integral;   // 定义积分值
} _pid;
extern _pid pid_speed, pid_speed2;
extern _pid pid_location, pid_location2;

void PID_param_init(void);
void set_pid_target(_pid *pid, float temp_val);
float get_pid_target(_pid *pid);
void set_p_i_d(_pid *pid, float p, float i, float d);

float location_pid_realize(_pid *pid, float actual_val);
float speed_pid_realize(_pid *pid, float actual_val);

#endif
