#include "OpenMV.h"
#include "stdio.h"
#include "usart.h"
#define buff_size 18
/*�ĸ��������ڴ��Ŀ����������������Լ���ȣ��߶�*/
int count = 0;
/*���ݽ��պ���*/
void Openmv_Receive_Data(uint8_t Com_Data)
{
	/*ѭ�������*/
	uint8_t i;
	/*��������*/
	static uint8_t RxCounter1 = 0; // ����
	/*���ݽ�������*/
	static uint16_t RxBuffer1[buff_size] = {0};
	/*���ݴ���״̬λ*/
	static uint8_t RxState = 0;
	/*�����ݽ���У׼���ж��Ƿ�Ϊ��Ч����*/
	if (RxState == 0 && Com_Data == 0x2C) // 0x2c֡ͷ
	{

		RxState = 1;
		RxBuffer1[RxCounter1++] = Com_Data;
	}

	else if (RxState == 1 && Com_Data == 0x12) // 0x12֡ͷ
	{
		RxState = 2;
		RxBuffer1[RxCounter1++] = Com_Data;
	}
	else if (RxState == 2)
	{

		RxBuffer1[RxCounter1++] = Com_Data;
		if (RxCounter1 >= buff_size || Com_Data == 0x5B) // RxBuffer1��������,�������ݽ���
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

	else if (RxState == 3) // ����Ƿ���ܵ�������־
	{
		if (RxBuffer1[RxCounter1 - 1] == 0x5B)
		{
			// RxFlag1 = 0;
			RxCounter1 = 0;
			RxState = 0;
		}
		else // ���մ���
		{
			RxState = 0;
			RxCounter1 = 0;
			for (i = 0; i < buff_size; i++)
			{
				RxBuffer1[i] = 0x00; // �����������������
			}
		}
	}

	else // �����쳣
	{
		RxState = 0;
		RxCounter1 = 0;
		for (i = 0; i < buff_size; i++)
		{
			RxBuffer1[i] = 0x00; // �����������������
		}
	}
}
