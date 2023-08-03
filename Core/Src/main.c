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
#define BUFFER_SIZE 16
#define RED_BUFFER 2
// 缓冲区用于存储接收到的数�???
uint8_t rxBuffer[BUFFER_SIZE];
uint32_t rxIndex = 0;
uint8_t redBuffer[RED_BUFFER];
uint32_t redIndex = 0;
// 舵机巡线
int targetX = 0;
int targetY = 0;
uint8_t buffer1[1];

int orign_x = 640;
int orign_y = 788;
int pwm_test_x;
int pwm_test_y;
// 接收红色�?光坐�?
int16_t redX;
int16_t redY;
// 接收黑框坐标
int16_t x_left_top;
int16_t y_left_top;
int16_t x_right_top;
int16_t y_right_top;
int16_t x_right_bottom;
int16_t y_right_bottom;
int16_t x_left_bottom;
int16_t y_left_bottom;

int flag_servo = 0;
int flag_problem = 0;
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
	TIM3->CCR1 = pwm_y;
	TIM3->CCR2 = pwm_x;
	current_x = pwm_x;
	current_y = pwm_y;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	pwm_test_x = orign_x;
	pwm_test_y = orign_y;

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
	setServoPwm(orign_x, orign_y);

	// 使能串口三接收中�???
	HAL_UART_Receive_IT(&huart3, &rxBuffer[rxIndex], 1);
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

		// if (flag_problem == 2)
		// {
		// 	Problem2();
		// 	flag_problem = 0;
		// }
		// else if (flag_problem == 3)
		// {
		// 	Problem3();
		// 	flag_problem = 0;
		// }
		if (flag_servo)
		{
			servo_pid_test(redX, redY, 0, 0);
			// servo_test(redX, redY);
			flag_servo = 0;
		}

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

void Problem1(void)
{
	setServoPwm(orign_x, orign_y);
}
void Problem2(void)
{
	// 顺时针移动
	// 左上顶点
	setServoPwm(750, 717);
	delay_ms(1000);
	// 右上顶点
	setServoPwm(601, 712);
	delay_ms(1000);
	// 右下顶点
	setServoPwm(605, 844);
	delay_ms(1000);
	// 左下顶点
	setServoPwm(745, 844);
	delay_ms(1000);
	// 回左上角
	setServoPwm(750, 717);
}
void Problem3(void)
{
	// 顺时针移动
	// 左上顶点
	setServoPwm(733, 765);
	delay_ms(1000);
	// 右上顶点
	setServoPwm(647, 758);
	delay_ms(1000);
	// 右下顶点
	setServoPwm(647, 818);
	delay_ms(1000);
	// 左下顶点
	setServoPwm(730, 818);
	delay_ms(1000);
	// 回左上角
	setServoPwm(733, 765);
}
// 串口三接收中断处理函�??
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart3)
	{
		// 读取接收到的数据，并存储到缓冲区�??
		rxBuffer[rxIndex++] = huart3.Instance->DR;

		// 判断是否接收完成
		if (rxIndex >= BUFFER_SIZE)
		{
			// 接收完成，解析X坐标和Y坐标
			redY = (int16_t)((rxBuffer[3] << 8) | rxBuffer[2]);
			redX = (int16_t)((rxBuffer[1] << 8) | rxBuffer[0]);

			// x_left_top = (int16_t)((rxBuffer[3] << 8) | rxBuffer[2]);
			// y_left_top = (int16_t)((rxBuffer[1] << 8) | rxBuffer[0]);

			// x_right_top = (int16_t)((rxBuffer[7] << 8) | rxBuffer[6]);
			// y_right_top = (int16_t)((rxBuffer[5] << 8) | rxBuffer[4]);

			// x_right_bottom = (int16_t)((rxBuffer[11] << 8) | rxBuffer[10]);
			// y_right_bottom = (int16_t)((rxBuffer[9] << 8) | rxBuffer[8]);

			// x_left_bottom = (int16_t)((rxBuffer[15] << 8) | rxBuffer[14]);
			// y_left_bottom = (int16_t)((rxBuffer[13] << 8) | rxBuffer[12]);

			// redX = (int16_t)((rxBuffer[19] << 8) | rxBuffer[18]);
			// redY = (int16_t)((rxBuffer[17] << 8) | rxBuffer[16]);

			// printf("%d  %d\r\n", x_left_bottom, y_left_bottom);

			printf("redX:%d  redY:%d\r\n", redX, redY);
			// 使用 receivedX �?? receivedY 进行后续处理
			// 重置缓冲区索引，准备下一次接�??
			flag_servo = 1;
			rxIndex = 0;
		}

		// 重新启用串口三接收中断，以继续接收数�??
		HAL_UART_Receive_IT(&huart3, &rxBuffer[rxIndex], 1);
	}
	// if (huart == &huart6)
	// {
	// 	// 读取接收到的数据，并存储到缓冲区�??
	// 	redBuffer[redIndex++] = huart2.Instance->DR;

	// 	// 判断是否接收完成
	// 	if (redIndex >= RED_BUFFER)
	// 	{
	// 		// 接收完成，解析X坐标和Y坐标
	// 		// receivedX = (int16_t)((rxBuffer[1] << 8) | rxBuffer[0]);
	// 		// receivedY = (int16_t)((rxBuffer[3] << 8) | rxBuffer[2]);

	// 		redX = (int16_t)((redBuffer[1] << 8));
	// 		redY = (int16_t)((redBuffer[0] << 8));

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
