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
	
	int left_pwm=speed;
	int right_pwm=speed;
   if (flag)
   {
//		motor_forward();
     if (flag==1) {		 
	car_stright(left_pwm, right_pwm);}
	 else if(flag==2){
    //    motor_turn_right();
	 car_stright(left_pwm*0.5, right_pwm);
	 }
	 else if(flag==3){
        //  motor_turn_left();
	 car_stright(left_pwm, right_pwm*0.5);
	 }
     else if(flag==4){
        //  motor_turn_right();
	 car_stright(left_pwm*0.2, right_pwm);
	 }
     else if(flag==5){
    //    motor_turn_left();
	 car_stright(left_pwm, right_pwm*0.2);
	 }
   }	 
// 	 else {
// //        car_wait();
// //    while (!CarRight90());
//     car_stop();
   
//     }
}


