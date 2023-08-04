#include "OpenMV.h"
#include "stdio.h"
#include "usart.h"
#define buff_size 18
/*四个变量用于存放目标物体的中心坐标以及宽度，高度*/
int count = 0;
/*数据接收函数*/
void Openmv_Receive_Data(uint8_t Com_Data)
{
	/*循环体变量*/
	uint8_t i;
	/*计数变量*/
	static uint8_t RxCounter1 = 0; // 计数
	/*数据接收数组*/
	static uint16_t RxBuffer1[buff_size] = {0};
	/*数据传输状态位*/
	static uint8_t RxState = 0;
	/*对数据进行校准，判断是否为有效数据*/
	if (RxState == 0 && Com_Data == 0x2C) // 0x2c帧头
	{

		RxState = 1;
		RxBuffer1[RxCounter1++] = Com_Data;
	}

	else if (RxState == 1 && Com_Data == 0x12) // 0x12帧头
	{
		RxState = 2;
		RxBuffer1[RxCounter1++] = Com_Data;
	}
	else if (RxState == 2)
	{

		RxBuffer1[RxCounter1++] = Com_Data;
		if (RxCounter1 >= buff_size || Com_Data == 0x5B) // RxBuffer1接受满了,接收数据结束
		{
			RxState = 3;
			if (count < 20)
			{
				x_left_top = RxBuffer1[RxCounter1 - 4];
				y_left_top = RxBuffer1[RxCounter1 - 2];
				x_right_top = RxBuffer1[RxCounter1 - 8];
				y_right_top = RxBuffer1[RxCounter1 - 6];
				x_right_bottom = RxBuffer1[RxCounter1 - 12];
				y_right_bottom = RxBuffer1[RxCounter1 - 10];
				x_left_bottom = RxBuffer1[RxCounter1 - 16];
				y_left_bottom = RxBuffer1[RxCounter1 - 14];
				count = count + 1;
				printf("%d  %d  %d  %d  %d  %d  %d  %d\r\n", x_left_bottom, y_left_bottom, x_right_bottom, y_right_bottom, x_right_top, y_right_top, x_left_top, y_left_top);
			}

			// printf("redX2:%d redY2:%d\r\n", redX, redY);

			flag_servo = 1;

			// printf("%d\r   ", redX);
			// printf("%d\r\n", redY);
		}
	}

	else if (RxState == 3) // 检测是否接受到结束标志
	{
		if (RxBuffer1[RxCounter1 - 1] == 0x5B)
		{
			// RxFlag1 = 0;
			RxCounter1 = 0;
			RxState = 0;
		}
		else // 接收错误
		{
			RxState = 0;
			RxCounter1 = 0;
			for (i = 0; i < buff_size; i++)
			{
				RxBuffer1[i] = 0x00; // 将存放数据数组清零
			}
		}
	}

	else // 接收异常
	{
		RxState = 0;
		RxCounter1 = 0;
		for (i = 0; i < buff_size; i++)
		{
			RxBuffer1[i] = 0x00; // 将存放数据数组清零
		}
	}
}
