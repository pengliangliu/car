#include "xunji.h"
#include  "main.h"
#include "stdio.h"
#include "tim.h"
#include "pid.h"
void readLEDsState(GPIO_PinState *ledStates)
{
    // 读取每个引脚的电平，并保存到数组中
    ledStates[0] = HAL_GPIO_ReadPin(LED0_GPIO_Port, LED0_Pin);
    ledStates[1] = HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin);
    ledStates[2] = HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin);
    ledStates[3] = HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin);
    ledStates[4] = HAL_GPIO_ReadPin(LED4_GPIO_Port, LED4_Pin);
    ledStates[5] = HAL_GPIO_ReadPin(LED5_GPIO_Port, LED5_Pin);
    ledStates[6] = HAL_GPIO_ReadPin(LED6_GPIO_Port, LED6_Pin);

				processLEDStates(ledStates);
}

int  processLEDStates(GPIO_PinState *ledStates)
{
	if (ledStates[0] == GPIO_PIN_SET && ledStates[1] == GPIO_PIN_SET && ledStates[2] == GPIO_PIN_SET &&
        ledStates[3] == GPIO_PIN_SET && ledStates[4] == GPIO_PIN_SET && ledStates[5] == GPIO_PIN_SET &&
        ledStates[6] == GPIO_PIN_SET)
    {
        return 1;
					
    }
		else 
		{
     // 判断ledStates[3]的值
    if (ledStates[3] == GPIO_PIN_SET)
    {
        // 判断ledStates[4]和ledStates[5]的值
        if (ledStates[4] == GPIO_PIN_RESET && ledStates[5] == GPIO_PIN_RESET)
        {
           CarStraight();
        }
    }
    
    if (ledStates[4] == GPIO_PIN_RESET && ledStates[2] == GPIO_PIN_SET)
       {
           CarStraight();
       }
    else if (ledStates[4] == GPIO_PIN_SET && ledStates[2] == GPIO_PIN_RESET)
       {
           CarStraight();
					 
       }

}
		// 判断ledStates全部为1时执行某个操作
    return 0;
}
