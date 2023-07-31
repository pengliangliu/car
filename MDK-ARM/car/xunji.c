#include "xunji.h"
#include  "main.h"
#include "stdio.h"
#include "tim.h"
#include "pid.h"
#include "car.h"


int readLEDsState(GPIO_PinState *ledStates)
{
    // 读取每个引脚的电平，并保存到数组中
    ledStates[0] = HAL_GPIO_ReadPin(LED0_GPIO_Port, LED0_Pin);
    ledStates[1] = HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin);
    ledStates[2] = HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin);
    ledStates[3] = HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin);
    ledStates[4] = HAL_GPIO_ReadPin(LED4_GPIO_Port, LED4_Pin);
    ledStates[5] = HAL_GPIO_ReadPin(LED5_GPIO_Port, LED5_Pin);
    ledStates[6] = HAL_GPIO_ReadPin(LED6_GPIO_Port, LED6_Pin);

			return	processLEDStates(ledStates);
}

int  processLEDStates(GPIO_PinState *ledStates)
{
	int i;
for (i = 0; i < 7; i++) {
    if (ledStates[i] != GPIO_PIN_SET) {
        break;
    }
}
if (i == 7) {
    return 7;
}
    
		
    //  判断ledStates[3]的值 _细线加上它,粗线注释掉
    if (ledStates[3] == GPIO_PIN_SET)
    {
        // 判断ledStates[4]和ledStates[5]的值     
        if (ledStates[4] == GPIO_PIN_RESET && ledStates[5] == GPIO_PIN_RESET)
        {
           return 1;
        }
    }

    //粗线
    // if (ledStates[4] == GPIO_PIN_SET && ledStates[5] == GPIO_PIN_SET)
    //     {
    //        return 1;
    //     }


    if (ledStates[4] == GPIO_PIN_RESET && ledStates[2] == GPIO_PIN_SET)
       {
           return 2;
       }
      if (ledStates[4] == GPIO_PIN_SET && ledStates[2] == GPIO_PIN_RESET)
       {
           return 3;
					 
       }
       if (ledStates[5] == GPIO_PIN_RESET && ledStates[1] == GPIO_PIN_SET)
       {
           return 4;
       }
     if (ledStates[5] == GPIO_PIN_SET && ledStates[1] == GPIO_PIN_RESET)
       {
           return 5;				 
       }
	return 0;
}

void track(int flag,int speed){
	
	int left_pwm;
	int right_pwm;
    if (flag) {
        if (flag == 1) {
            left_pwm = speed;
            right_pwm = speed;
					track_pid(left_pwm, right_pwm);	
        } else if (flag == 2) {
            left_pwm = speed * 0.5;
            right_pwm = speed;     
track_pid(left_pwm, right_pwm);						
        } else if (flag == 3) {
            left_pwm = speed;
            right_pwm = speed * 0.5;
					track_pid(left_pwm, right_pwm);	
            
        } else if (flag == 4) {
            left_pwm = speed * 0.2;
            right_pwm = speed;
					track_pid(left_pwm, right_pwm);	
        } else if (flag == 5) {
            left_pwm = speed;
            right_pwm = speed * 0.2;
					track_pid(left_pwm, right_pwm);	
        }
        
    }
// 	 else {
// //        car_wait();
// //    while (!CarRight90());
//     car_stop();
   
//     }
}

PID_Controller pid_track_left;
PID_Controller pid_track_right;
void track_pid(int target_left,int target_right) {
    int current_leftpwm=HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
		int current_rightpwm=HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
	printf("left:%d,right:%d\r\n",current_leftpwm,current_rightpwm);
		pid_init(&pid_track_left, target_left, 0.45, 0, 0.015);
		pid_init(&pid_track_right, target_right, 0.45, 0, 0.015);
		int control_left = pid_control(&pid_track_left, current_leftpwm);
		int control_right = pid_control(&pid_track_right, current_rightpwm);
		printf("conleft:%d,conright:%d\r\n",control_left,control_right);
		current_leftpwm=control_left+current_leftpwm;
		current_rightpwm=control_right+current_rightpwm;
	// Apply limits
    if (current_leftpwm > 500) {
        current_leftpwm = 500;
    } else if (current_leftpwm < 100) {
        current_leftpwm = 100;
    }

    if (current_rightpwm > 500) {
        current_rightpwm = 500;
    } else if (current_rightpwm < 100) {
        current_rightpwm = 100;
    }
		printf("left:%d,right:%d\r\n",current_leftpwm,current_rightpwm);	
		car_stright(current_leftpwm, current_rightpwm);
	
	
}

//int calculate_pid_left(int target_left, int current_left) {
//    static PID_Controller pid_track_left;
//    pid_init(&pid_track_left, target_left, 1.0, 0, 0.01);
//    int control_left = pid_control(&pid_track_left, current_left);
//    return control_left;
//}

//// Function to calculate PID control value for right track
//int calculate_pid_right(int target_right, int current_right) {
//    static PID_Controller pid_track_right;
//    pid_init(&pid_track_right, target_right, 1.0, 0, 0.01);
//    int control_right = pid_control(&pid_track_right, current_right);
//    return control_right;
//}
