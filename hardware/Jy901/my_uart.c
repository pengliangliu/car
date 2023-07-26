#include "my_uart.h"

User_USART JY901_data;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;

struct SAcc stcAcc;
struct SGyro stcGyro;
struct SAngle stcAngle;

uint8_t a = 0;


// ��ʼ������
void User_USART_Init(User_USART *Data)
{
	for (uint16_t i = 0; i < RXBUFFER_LEN; i++)
		Data->RxBuffer[i] = 0;
	Data->frame_head = 0x55;
	Data->Rx_flag = 0;
	Data->Rx_len = 0;
}

void JY901_Process()
{
	if (JY901_data.Rx_len < RXBUFFER_LEN)
		return; // ���ݳ��Ȳ���

	for (uint8_t i = 0; i < 4; i++)
	{
		if (JY901_data.RxBuffer[i * 11] != JY901_data.frame_head)
			continue;
		switch (JY901_data.RxBuffer[i * 11 + 1])
		{
		case 0x51:
			memcpy(&stcAcc, &JY901_data.RxBuffer[2 + i * 11], 8);
			for (uint8_t j = 0; j < 3; j++)
				JY901_data.acc.a[j] = (float)stcAcc.a[j] / 32768 * 16;
			// printf("acc:%f,%f,%f", JY901_data.acc.a[0], JY901_data.acc.a[1], JY901_data.acc.a[2]); // ���ٶ�
			break;
		case 0x52:
			memcpy(&stcGyro, &JY901_data.RxBuffer[2 + i * 11], 8);
			for (uint8_t j = 0; j < 3; j++)
				JY901_data.w.w[j] = (float)stcGyro.w[j] / 32768 * 2000;
			// printf("w:%f,%f,%f", JY901_data.w.w[0], JY901_data.w.w[1], JY901_data.w.w[2]); // ���ٶ�
			// ���ٶ�
			break;
		case 0x53:
			memcpy(&stcAngle, &JY901_data.RxBuffer[2 + i * 11], 8);
			for (uint8_t j = 0; j < 3; j++)
				JY901_data.angle.angle[j] = (float)stcAngle.Angle[j] / 32768 * 180;
			printf("angle:%f,%f,%f\r\n", JY901_data.angle.angle[0], JY901_data.angle.angle[1], JY901_data.angle.angle[2]); // ���ٶ�
																														   // �Ƕ�
			break;
		}
	}
}
