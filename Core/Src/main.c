/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "OpenMV.h"
#include "xunji.h"
#include "mpu6050.h"
#include "delay.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "pid.h"
#include "car.h"
#include "oled.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BUFFER_SIZE 20
#define RED_BUFFER 4
// 缓冲区用于存储接收到的数�???
uint8_t rxBuffer;
uint32_t rxIndex = 0;
uint8_t redBuffer[RED_BUFFER];
uint32_t redIndex = 0;
// 舵机巡线
int targetX = 0;
int targetY = 0;
uint8_t buffer1[1];

int pwm_orign_x = 655;
int pwm_orign_y = 776;
int pwm_test_x;
int pwm_test_y;
// 接收红色�?光坐�?
int16_t redX = 86;
int16_t redY = 54;
// 接收黑框坐标
int16_t x_left_top = 56;
int16_t y_left_top = 40;
int16_t x_right_top = 117;
int16_t y_right_top = 36;
int16_t x_right_bottom = 121;
int16_t y_right_bottom = 82;
int16_t x_left_bottom = 57;
int16_t y_left_bottom = 85;

int flag_servo = 0;
int flag_problem = 0;

int receivedData;
char *endPtr;
char *token;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE BEGIN PV */
void Problem1(void);
void Problem2(void);
void Problem3(void);
void Problem4(void);
void TrackX(int x1, int x2, int y1, int y2);
void TrackY(int x1, int x2, int y1, int y2);
/* USER CODE END PV */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
	逆时针增�???
	x[0,180]-> PWM[500,3500]
	y[0,90]-> PWM[1500,2500]
*/
void setServoPosition(int angle_x, int angle_y)
{

	// // 舵机
	uint16_t pulse_x = ((angle_x * 2500) / 270) + 499;
	uint16_t pulse_y = ((angle_y * 1000) / 120) + 1499;

	TIM3->CCR1 = pulse_y;
	TIM3->CCR2 = pulse_x;
}

void setServoPwm(int pwm_x, int pwm_y)
{
	if (pwm_x > 900)
		pwm_x = 900;
	else if (pwm_x < 400)
		pwm_x = 400;
	if (pwm_y > 900)
		pwm_y = 900;
	else if (pwm_y < 400)
		pwm_y = 400;
	TIM3->CCR1 = pwm_y;
	TIM3->CCR2 = pwm_x;
	current_x = pwm_x;
	current_y = pwm_y;
	// printf("current_x:%d,current_x:%d\r\n", current_x, current_y);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	pwm_test_x = pwm_orign_x;
	pwm_test_y = pwm_orign_y;

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART6_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	// setServoPwm(509, 500);
	// [299, 1199]
	setServoPwm(pwm_orign_x, pwm_orign_y);

	// 使能串口三接收中�???
	HAL_UART_Receive_IT(&huart3, (void *)&rxBuffer, 1);
	HAL_UART_Receive_IT(&huart1, &buffer1[1], 1);
	// HAL_UART_Receive_IT(&huart2, &redBuffer[redIndex], 1);
	HAL_UART_Receive_IT(&huart6, &redBuffer[redIndex], 1);

	//	Mpu6050_Init();

	//	OLED_Init();
	//	OLED_Clear();

	// HAL_ADC_Start_IT(&hadc1);
	printf("start\r\n");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if (flag_problem == 2)
		{
			Problem2();
			flag_problem = 0;
		}
		else if (flag_problem == 3)
		{
			Problem3();
			flag_problem = 0;
		}
		else if (flag_problem == 4)
		{
			Problem4();
			flag_problem = 0;
		}
		// if (flag_servo)
		// {

		// 	// servo_pid_test(redX, redY, 80, 60);
		// 	servo_test(redX, redY);

		// 	// printf("x_left_top:%d y_left_top:%d \r\n", x_left_top, y_left_top);
		// 	// printf("redX1:%d redY1:%d\r\n", redX, redY);
		// 	flag_servo = 0;
		// }

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void TrackX(int x1, int x2, int y1, int y2)
{
	int countX = 0;
	countX = x1 - x2;
	for (int i = 0; i < fabs(countX); i++)
	{
		if (countX > 0)
			x1 = x1 - 1;
		else if (countX < 0)
			x1 = x1 + 1;
		setServoPwm(x1, (y1 + y2) / 2);
		delay_ms(20);
	}
}
void TrackY(int x1, int x2, int y1, int y2)
{
	int countY = 0;
	countY = y1 - y2;
	for (int i = 0; i < fabs(countY); i++)
	{
		if (countY > 0)
			y1 = y1 - 1;
		else if (countY < 0)
			y1 = y1 + 1;
		setServoPwm((x1 + x2) / 2, y1);
		delay_ms(20);
	}
}

void TrackXY(int x1, int x2, int y1, int y2)
{
	int countX = x1 - x2;
	int countY = y1 - y2;
	int minCount = countX < countY ? countY : countX; // 取最大次数
	int stepX = (int)countX / minCount;
	int stepY = (int)countY / minCount + 1;
	for (int i = 0; i < fabs(minCount); i++)
	{
		if (countX > 0)
			x1 = x1 - stepX;
		else if (countX < 0)
			x1 = x1 + stepX;
		if (countY > 0)
			y1 = y1 - stepY;
		else if (countY < 0)
			y1 = y1 + stepY;
		setServoPwm(x1, y1);
		delay_ms(20);
	}
}
void Problem1(void)
{
	setServoPwm(pwm_orign_x, pwm_orign_y);
}
void Problem2(void)
{
	int x0, y0;
	int x1, y1;
	int x2, y2;
	int x3, y3;
	int x4, y4;
	x1 = 708;
	y1 = 724;
	x2 = 602;
	y2 = 724;
	x3 = 602;
	y3 = 828;
	x4 = 708;
	y4 = 828;
	x0 = x1;
	y0 = y1;
	// 顺时针移动
	// 左上顶点
	setServoPwm(x1, y1);
	delay_ms(1000);
	TrackX(x1, x2, y1, y2);
	// 右上顶点
	setServoPwm(x2, y2);
	delay_ms(100);
	TrackY(x2, x3, y2, y3);
	// 右下顶点
	setServoPwm(x3, y3);
	delay_ms(100);
	TrackX(x3, x4, y3, y4);
	// 左下顶点
	setServoPwm(x4, y4);
	delay_ms(100);
	TrackY(x4, x1, y4, y1);
	// 回左上角
	setServoPwm(x0, y0);
}
void Problem3(void)
{
	int x0, y0;
	int x1, y1;
	int x2, y2;
	int x3, y3;
	int x4, y4;
	x1 = 687;
	y1 = 754;
	x2 = 628;
	y2 = 750;
	x3 = 624;
	y3 = 794;
	x4 = 685;
	y4 = 798;
	x0 = x1;
	y0 = y1;
	// 顺时针移动
	// 左上顶点
	setServoPwm(x1, y1);
	delay_ms(100);
	TrackX(x1, x2, y1, y2);
	// 右上顶点
	setServoPwm(x2, y2);
	delay_ms(100);
	TrackY(x2, x3, y2, y3);
	// 右下顶点
	setServoPwm(x3, y3);
	delay_ms(100);
	TrackX(x3, x4, y3, y4);
	// 左下顶点
	setServoPwm(x4, y4);
	delay_ms(100);
	TrackY(x4, x1, y4, y1);
	// 回左上角
	setServoPwm(x0, y0);
}
void Problem4(void)
{
	printf("x_left_top:%d,y_left_top:%d,x_right_top:%d,y_right_top:%d", x_left_top, y_left_top, x_right_top, y_right_top);
	int x_left_top_track = -3;
	int y_left_top_track = -3;
	int x_right_top_track = 5;
	int y_right_top_track = 5;
	int x_right_bottom_track = 2;
	int y_right_bottom_track = -2;
	int x_left_bottom_track = -4;
	int y_left_bottom_track = -4;

	float pwm_rate_x = 0.95;
	float pwm_rate_y = 1;

	int x_left_top_error = redX - x_left_top;
	int y_left_top_error = redY - y_left_top;

	int x_right_top_error = x_left_top - x_right_top;
	int y_right_top_error = y_left_top - y_right_top;

	int x_right_bottom_error = x_right_top - x_right_bottom;
	int y_right_bottom_error = y_right_top - y_right_bottom;

	int x_left_bottom_error = x_right_bottom - x_left_bottom;
	int y_left_bottom_error = y_right_bottom - y_left_bottom;

	int pwm_x_left_top;
	int pwm_y_left_top;
	int pwm_x_right_top;
	int pwm_y_right_top;
	int pwm_x_right_bottom;
	int pwm_y_right_bottom;
	int pwm_x_left_bottom;
	int pwm_y_left_bottom;

	if (fabs(y_right_top_error) < 16)
	{
		x_left_top_track = -4;
		y_left_top_track = -4;
		x_right_top_track = 7;
		y_right_top_track = 7;
		x_right_bottom_track = 2;
		y_right_bottom_track = -4;
		x_left_bottom_track = -9;
		y_left_bottom_track = -4;
	}
	else if (fabs(y_right_top_error) < 26)
	{
		x_left_top_track = -3;
		y_left_top_track = -4;
		x_right_top_track = 4;
		y_right_top_track = 7;
		x_right_bottom_track = 4;
		y_right_bottom_track = -6;
		x_left_bottom_track = -8;
		y_left_bottom_track = -2;
	}
	// else if (fabs(y_right_top_error) < 34)
	// {
	// 	int x_left_top_track = -3;
	// 	int y_left_top_track = -3;
	// 	int x_right_top_track = 5;
	// 	int y_right_top_track = 5;
	// 	int x_right_bottom_track = 2;
	// 	int y_right_bottom_track = -2;
	// 	int x_left_bottom_track = -4;
	// 	int y_left_bottom_track = -4;
	// }
	pwm_x_left_top = pwm_orign_x + (int)x_left_top_error / pwm_rate_x + x_left_top_track;
	pwm_y_left_top = pwm_orign_y - (int)y_left_top_error / pwm_rate_y + y_left_top_track;

	pwm_x_right_top = pwm_x_left_top + (int)x_right_top_error / pwm_rate_x + x_right_top_track;
	pwm_y_right_top = pwm_y_left_top - (int)y_right_top_error / pwm_rate_y + y_right_top_track;

	pwm_x_right_bottom = pwm_x_right_top + (int)x_right_bottom_error / pwm_rate_x + x_right_bottom_track;
	pwm_y_right_bottom = pwm_y_right_top - (int)y_right_bottom_error / pwm_rate_y + y_right_bottom_track;

	pwm_x_left_bottom = pwm_x_right_bottom + (int)x_left_bottom_error / pwm_rate_x + x_left_bottom_track;
	pwm_y_left_bottom = pwm_y_right_bottom - (int)y_left_bottom_error / pwm_rate_y + y_left_bottom_track;

	// printf("x_left_top_error:%d,y_left_top_error:%d\r\n", x_left_top_error, y_left_top_error);
	// printf("x_right_top_error:%d,y_right_top_error:%d\r\n", x_right_top_error, y_right_top_error);
	// printf("x_right_bottom_error:%d,y_right_bottom_error:%d\r\n", x_right_bottom_error, y_right_bottom_error);
	// printf("x_left_bottom_error:%d,y_left_bottom_error:%d\r\n", x_left_bottom_error, y_left_bottom_error);

	// printf("pwm_x_left_top:%d,pwm_y_left_top:%d\r\n", pwm_x_left_top, pwm_y_left_top);
	// printf("pwm_x_right_top:%d,pwm_y_right_top:%d\r\n", pwm_x_right_top, pwm_y_right_top);
	// printf("pwm_x_right_bottom:%d,pwm_y_right_bottom:%d\r\n", pwm_x_right_bottom, pwm_y_right_bottom);
	// printf("pwm_x_left_bottom:%d,pwm_y_left_bottom:%d\r\n", pwm_x_left_bottom, pwm_y_left_bottom);

	setServoPwm(pwm_x_left_top, pwm_y_left_top);
	delay_ms(1000);
	// TrackXY(pwm_x_left_top, pwm_x_right_top, pwm_y_left_top, pwm_y_right_top);
	setServoPwm(pwm_x_right_top, pwm_y_right_top);
	delay_ms(1000);
	setServoPwm(pwm_x_right_bottom, pwm_y_right_bottom);
	delay_ms(1000);
	setServoPwm(pwm_x_left_bottom, pwm_y_left_bottom);
	delay_ms(1000);
	setServoPwm(pwm_x_left_top, pwm_y_left_top);
	delay_ms(1000);
}

// 串口三接收中断处理函�??
uint8_t tempt;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart == &huart3)
	{
		tempt = rxBuffer;
		Openmv_Receive_Data(tempt);
		// 读取接收到的数据，并存储到缓冲区�??
		// rxBuffer[rxIndex++] = huart3.Instance->DR;

		// // 判断是否接收完成
		// if (rxIndex >= BUFFER_SIZE)
		// {
		// 	// 接收完成，解析X坐标和Y坐标
		// 	// redY = (int16_t)((rxBuffer[3] << 8) | rxBuffer[2]);
		// 	// redX = (int16_t)((rxBuffer[1] << 8) | rxBuffer[0]);
		// 	int x=atoi(rxBuffer);
		// 	printf("%d\r\n", x);

		// 	// x_left_top = (int16_t)((rxBuffer[1] << 8) | rxBuffer[0]);
		// 	// y_left_top = (int16_t)((rxBuffer[3] << 8) | rxBuffer[2]);

		// 	// x_right_top = (int16_t)((rxBuffer[5] << 8) | rxBuffer[4]);
		// 	// y_right_top = (int16_t)((rxBuffer[7] << 8) | rxBuffer[6]);

		// 	// x_right_bottom = (int16_t)((rxBuffer[9] << 8) | rxBuffer[8]);
		// 	// y_right_bottom = (int16_t)((rxBuffer[11] << 8) | rxBuffer[10]);

		// 	// x_left_bottom = (int16_t)((rxBuffer[13] << 8) | rxBuffer[12]);
		// 	// y_left_bottom = (int16_t)((rxBuffer[15] << 8) | rxBuffer[14]);

		// 	// redX = (int16_t)((rxBuffer[17] << 8) | rxBuffer[16]);
		// 	// redY = (int16_t)((rxBuffer[19] << 8) | rxBuffer[18]);

		// 	// printf("%d  %d\r\n", x_left_bottom, y_left_bottom);

		// 	// printf("x_left_top:%d \r\n", x_left_top);
		// 	// printf("redX:%d redY:%d\r\n", redX, redY);
		// 	// 使用 receivedX �?? receivedY 进行后续处理
		// 	// 重置缓冲区索引，准备下一次接�??
		// 	flag_servo = 1;
		// 	rxIndex = 0;
		// }

		// 重新启用串口三接收中断，以继续接收数�??
		HAL_UART_Receive_IT(&huart3, (void *)&rxBuffer, 1);
	}
	// if (huart == &huart6)
	// {
	// 	// 读取接收到的数据，并存储到缓冲区�??
	// 	redBuffer[redIndex++] = huart6.Instance->DR;

	// 	// 判断是否接收完成
	// 	if (redIndex >= RED_BUFFER)
	// 	{
	// 		// 接收完成，解析X坐标和Y坐标

	// 		redY = (int16_t)((rxBuffer[3] << 8) | rxBuffer[2]);
	// 		redX = (int16_t)((rxBuffer[1] << 8) | rxBuffer[0]);

	// 		printf("redX:%d, redY:%d\r\n", redX, redY);

	// 		redIndex = 0;
	// 	}
	// 	// 重新启用串口三接收中断，以继续接收数�??
	// 	HAL_UART_Receive_IT(&huart6, &redBuffer[redIndex], 1);
	// }
	else if (huart == &huart1)
	{
		// x 左+ 右-
		// y 下+ 上-
		buffer1[0] = huart->Instance->DR; // Read received data from UART DR register
		if (buffer1[0] == 0x00)
		{
			setServoPwm(675, 783);
			printf("reset\r\n");
		}
		// x轴 +-10
		else if (buffer1[0] == 0x01)
		{
			pwm_test_x = pwm_test_x + 10;
			printf("pwm_test_x:%d\r\n", pwm_test_x);
		}
		else if (buffer1[0] == 0x02)
		{
			pwm_test_x = pwm_test_x - 10;
			printf("pwm_test_x:%d\r\n", pwm_test_x);
		}
		// y轴 +-10
		else if (buffer1[0] == 0x03)
		{
			pwm_test_y = pwm_test_y + 10;
			printf("pwm_test_y:%d\r\n", pwm_test_y);
		}
		else if (buffer1[0] == 0x04)
		{
			pwm_test_y = pwm_test_y - 10;
			printf("pwm_test_y:%d\r\n", pwm_test_y);
		}
		// x轴 +-100
		else if (buffer1[0] == 0x05)
		{
			pwm_test_x = pwm_test_x + 100;
			printf("pwm_test_x:%d\r\n", pwm_test_x);
		}
		else if (buffer1[0] == 0x06)
		{
			pwm_test_x = pwm_test_x - 100;
			printf("pwm_test_x:%d\r\n", pwm_test_x);
		}
		// y轴 +-100
		else if (buffer1[0] == 0x07)
		{
			pwm_test_y = pwm_test_y + 100;
			printf("pwm_test_y:%d\r\n", pwm_test_y);
		}
		else if (buffer1[0] == 0x08)
		{
			pwm_test_y = pwm_test_y - 100;
			printf("pwm_test_y:%d\r\n", pwm_test_y);
		}
		// x轴 +-1
		else if (buffer1[0] == 0x09)
		{
			pwm_test_x = pwm_test_x + 1;
			printf("pwm_test_x:%d\r\n", pwm_test_x);
		}
		else if (buffer1[0] == 0x10)
		{
			pwm_test_x = pwm_test_x - 1;
			printf("pwm_test_x:%d\r\n", pwm_test_x);
		}
		// y轴 +-1
		else if (buffer1[0] == 0x12)
		{
			pwm_test_y = pwm_test_y + 1;
			printf("pwm_test_y:%d\r\n", pwm_test_y);
		}
		else if (buffer1[0] == 0x13)
		{
			pwm_test_y = pwm_test_y - 1;
			printf("pwm_test_y:%d\r\n", pwm_test_y);
		}
		else if (buffer1[0] == 0x14)
		{
			pwm_test_x = pwm_test_x + 50;
			printf("pwm_test_x:%d\r\n", pwm_test_x);
		}
		else if (buffer1[0] == 0x15)
		{
			pwm_test_x = pwm_test_x - 50;
			printf("pwm_test_x:%d\r\n", pwm_test_x);
		}
		// y轴 +-1
		else if (buffer1[0] == 0x16)
		{
			pwm_test_y = pwm_test_y + 50;
			printf("pwm_test_y:%d\r\n", pwm_test_y);
		}
		else if (buffer1[0] == 0x17)
		{
			pwm_test_y = pwm_test_y - 50;
			printf("pwm_test_y:%d\r\n", pwm_test_y);
		}
		else if (buffer1[0] == 0x90)
		{
			flag_problem = 2;
			printf("Problem2:\r\n");
		}
		else if (buffer1[0] == 0x91)
		{
			flag_problem = 3;
			printf("Problem3:\r\n");
		}
		else if (buffer1[0] == 0x92)
		{
			flag_problem = 4;
			printf("Problem4:\r\n");
		}
		else if (buffer1[0] == 0x98)
		{
			printf("current  x_left_top:%d, y_left_top:%d,x_right_top:%d,y_right_top:%d,x_right_bottom:%d,y_right_bottom:%d,x_left_bottom:%d,y_left_bottom:%d\r\n", x_left_top, y_left_top, x_right_top, y_right_top, x_right_bottom, y_right_bottom, x_left_bottom, y_left_bottom);
		}
		else if (buffer1[0] == 0x99)
		{
			printf("pwm_test_x:%d, pwm_test_y:%d\r\n", pwm_test_x, pwm_test_y);
		}
		// 重新启用串口接收中断，以继续接收数据
		setServoPwm(pwm_test_x, pwm_test_y);
		HAL_UART_Receive_IT(&huart1, &buffer1[1], 1);
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
