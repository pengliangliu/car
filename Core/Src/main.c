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
#include "stmflash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BUFFER_SIZE 20
#define RED_BUFFER 4
// 数字键盘相关
#define FLASH_SAVE_ADDR 0X08010000
// 定义行和列的数量
#define NUM_ROWS 4
#define NUM_COLS 4
// 行和列对应的IO�???
GPIO_TypeDef *row_ports[NUM_ROWS] = {GPIOA, GPIOC, GPIOA, GPIOA};
uint16_t row_pins[NUM_ROWS] = {GPIO_PIN_2, GPIO_PIN_2, GPIO_PIN_4, GPIO_PIN_7};

GPIO_TypeDef *col_ports[NUM_COLS] = {GPIOC, GPIOB, GPIOF, GPIOF};
uint16_t col_pins[NUM_COLS] = {GPIO_PIN_5, GPIO_PIN_1, GPIO_PIN_11, GPIO_PIN_7};
uint32_t load[20] = {655, 774,
					 708, 724, 602, 724, 602, 828, 708, 828,
					 708, 724, 602, 724, 602, 828, 708, 828};

int keyNumbers[NUM_ROWS][NUM_COLS] = {
	{1, 2, 3, 4},
	{5, 6, 7, 8},
	{9, 10, 11, 12},
	{13, 14, 15, 16}};

int flag_stop = 0;
// 缓冲区用于存储接收到的数�??????
uint8_t rxBuffer;
uint32_t rxIndex = 0;
uint8_t redBuffer[RED_BUFFER];
uint32_t redIndex = 0;
// 舵机巡线
int targetX = 0;
int targetY = 0;
uint8_t buffer1[1];

int pwm_orign_x;
int pwm_orign_y;
int pwm_test_x;
int pwm_test_y;
// 接收红色�????光坐�????
int16_t redX = 86;
int16_t redY = 54;
int16_t origin_x = 87;
int16_t origin_y = 54;
// 接收黑框坐标
int16_t x_left_top = 56;
int16_t y_left_top = 40;
int16_t x_right_top = 118;
int16_t y_right_top = 40;
int16_t x_right_bottom = 118;
int16_t y_right_bottom = 86;
int16_t x_left_bottom = 56;
int16_t y_left_bottom = 86;

int pwm_x_left_top_p2 = 708;
int pwm_y_left_top_p2 = 724;
int pwm_x_right_top_p2 = 602;
int pwm_y_right_top_p2 = 724;
int pwm_x_right_bottom_p2 = 602;
int pwm_y_right_bottom_p2 = 828;
int pwm_x_left_bottom_p2 = 708;
int pwm_y_left_bottom_p2 = 828;

int pwm_x_left_top_p3 = 708;
int pwm_y_left_top_p3 = 724;
int pwm_x_right_top_p3 = 602;
int pwm_y_right_top_p3 = 724;
int pwm_x_right_bottom_p3 = 602;
int pwm_y_right_bottom_p3 = 828;
int pwm_x_left_bottom_p3 = 708;
int pwm_y_left_bottom_p3 = 828;

int16_t rect_orign_x;
int16_t rect_orign_y;

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
void ButtonDebug(int pressedKey);
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
#define DEBOUNCE_DELAY_MS 20 // 调整此�?�以适应您的�???�???
int isButtonPressed(GPIO_TypeDef *port, uint16_t pin)
{
	uint32_t startTime = HAL_GetTick(); // 获取当前时间

	while (HAL_GetTick() - startTime < DEBOUNCE_DELAY_MS)
	{
		if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET)
		{
			// 按键仍然按下
			HAL_Delay(10); // 延迟�???小段时间，确保按键稳�???
		}
		else
		{
			// 按键已经释放
			return 0;
		}
	}

	// 在一段时间内保持按下状�?�，认为按键按下
	return 1;
}
int currentLevel = 0;
int previousLevel = 0;
int currentMenu = 0; // 当前�???在菜�???
int model = 1;
int scanKeyMatrix(void)
{
	int pressedKey = 0; // 默认值表示没有按键按�???
	unsigned char a, b, c;
	for (int i = 0; i < NUM_COLS; i++)
	{
		// 设置当前列为低电�???
		HAL_GPIO_WritePin(col_ports[i], col_pins[i], GPIO_PIN_RESET);
		for (c = 234; c > 0; c--)
			for (b = 123; b > 0; b--)
				for (a = 7; a > 0; a--)
					;
		// �???查每�???行的状�??
		for (int j = 0; j < NUM_ROWS; j++)
		{
			if (HAL_GPIO_ReadPin(row_ports[j], row_pins[j]) == GPIO_PIN_RESET)
			{
				pressedKey = keyNumbers[j][i]; // 获取按键编号
			}
		}

		// 恢复当前列为高电�???
		HAL_GPIO_WritePin(col_ports[i], col_pins[i], GPIO_PIN_SET);
	}

	return pressedKey;
}
/*
	逆时针增�??????
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

void BufferInit()
{
	// STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t *)load, sizeof(load) / sizeof(uint32_t));
	STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof(uint32_t));
	pwm_orign_x = load[0];
	pwm_orign_y = load[1];
	pwm_x_left_top_p2 = load[2];
	pwm_y_left_top_p2 = load[3];
	pwm_x_right_top_p2 = load[4];
	pwm_y_right_top_p2 = load[5];
	pwm_x_right_bottom_p2 = load[6];
	pwm_y_right_bottom_p2 = load[7];
	pwm_x_left_bottom_p2 = load[8];
	pwm_y_left_bottom_p2 = load[9];
	pwm_x_left_top_p3 = load[10];
	pwm_y_left_top_p3 = load[11];
	pwm_x_right_top_p3 = load[12];
	pwm_y_right_top_p3 = load[13];
	pwm_x_right_bottom_p3 = load[14];
	pwm_y_right_bottom_p3 = load[15];
	pwm_x_left_bottom_p3 = load[16];
	pwm_y_left_bottom_p3 = load[17];
	printf("pwm_orign_x:%d\r\n", pwm_orign_x);
	printf("pwm_orign_y:%d\r\n", pwm_orign_y);
	printf("pwm_x_left_top_p2:%d\r\n", pwm_x_left_top_p2);
	printf("pwm_y_left_top_p2:%d\r\n", pwm_y_left_top_p2);
	printf("pwm_x_right_top_p2:%d\r\n", pwm_x_right_top_p2);
	printf("pwm_y_right_top_p2:%d\r\n", pwm_y_right_top_p2);
	printf("pwm_x_right_bottom_p2:%d\r\n", pwm_x_right_bottom_p2);
	printf("pwm_y_right_bottom_p2:%d\r\n", pwm_y_right_bottom_p2);
	printf("pwm_x_left_bottom_p2:%d\r\n", pwm_x_left_bottom_p2);
	printf("pwm_y_left_bottom_p2:%d\r\n", pwm_y_left_bottom_p2);
	printf("pwm_x_left_top_p3:%d\r\n", pwm_x_left_top_p3);
	printf("pwm_y_left_top_p3:%d\r\n", pwm_y_left_top_p3);
	printf("pwm_x_right_top_p3:%d\r\n", pwm_x_right_top_p3);
	printf("pwm_y_right_top_p3:%d\r\n", pwm_y_right_top_p3);
	printf("pwm_x_right_bottom_p3:%d\r\n", pwm_x_right_bottom_p3);
	printf("pwm_y_right_bottom_p3:%d\r\n", pwm_y_right_bottom_p3);
	printf("pwm_x_left_bottom_p3:%d\r\n", pwm_x_left_bottom_p3);
	printf("pwm_y_left_bottom_p3:%d\r\n", pwm_y_left_bottom_p3);
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
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART6_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);
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

	BufferInit();
	// setServoPwm(509, 500);
	// [299, 1199]

	// 使能串口三接收中�??????
	HAL_UART_Receive_IT(&huart3, (void *)&rxBuffer, 1);
	HAL_UART_Receive_IT(&huart1, &buffer1[1], 1);
	// HAL_UART_Receive_IT(&huart2, &redBuffer[redIndex], 1);
	HAL_UART_Receive_IT(&huart6, &redBuffer[redIndex], 1);

	//	Mpu6050_Init();

	OLED_Init();
	OLED_Clear();

	// HAL_ADC_Start_IT(&hadc1);
	printf("start\r\n");
	setServoPwm(651, 778);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		int pressedKey = scanKeyMatrix();
		if (pressedKey == 6)
		{
			flag_problem = 1;
		}
		else if (pressedKey == 12)
		{
			flag_problem = 2;
		}
		else if (pressedKey == 14)
		{
			flag_problem = 3;
		}
		else if (pressedKey == 15)
		{
			flag_problem = 4;
		}
		ButtonDebug(pressedKey);
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
		// else if (flag_problem == 4)
		// {
		// 	Problem4();
		// 	flag_problem = 0;
		// }

		if (flag_problem == 1)
		{
			Problem1();
			flag_problem = 0;
		}
		else if (flag_problem == 2)
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
	}

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

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
		if (i == fabs(countX))
		{
			if (x1 > x2)
				x1 = x2;
			else if (x1 < x2)
				x1 = x2;
		}
		setServoPwm(x1, y2);
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
		if (i == fabs(countY) - 1)
		{
			if (y1 > y2)
				y1 = y2;
			else if (y1 < y2)
				y1 = y2;
		}

		setServoPwm(x2, y1);
		delay_ms(20);
	}
}

void TrackXY(int x1, int x2, int y1, int y2)
{
	int countX = x1 - x2;
	int countY = y1 - y2;
	int flagX = countX > 0 ? -1 : 1;
	int flagY = countY > 0 ? -1 : 1;
	printf("flagX:%d,flagY:%d\r\n", flagX, flagY);
	int minCount = countX < countY ? countY : countX; // 取最大次�???
	float stepX = abs(countX / (minCount * 1.0));
	float stepY = abs(countY / (minCount * 1.0));
	int x_point = abs((int)(stepX * 10) % 10);
	int y_point = abs((int)(stepY * 10) % 10);
	printf("x1:%d,x2:%d,y1:%d,y2:%d\r\n", x1, x2, y1, y2);
	printf("minCount:%d,stepX:%.3f,stepY:%.3f\r\n", minCount, stepX, stepY);
	printf("x_point:%d,y_point:%d\r\n", x_point, y_point);
	for (int i = 0; i < fabs(minCount) / 2; i++)
	{
		if (stepX == 1.0f)
		{
			x1 = x1 + (int)stepX * flagX;
			// 小数部分大于0.5
			if (y_point > 5)
			{
				if (i % 2 == 0)
				{
					y1 = y1 + (int)stepY * flagY;
				}
				else
				{
					y1 = y1 + ((int)stepY - 1) * flagY;
				}
			}
			else
			{
				if (i % 2 == 0)
				{
					y1 = y1 + (int)stepY * flagY;
				}
				else
				{
					y1 = y1 + ((int)stepY + 1) * flagY;
				}
			}
		}
		else if (stepY == 1.0f)
		{
			y1 = y1 + (int)stepY * flagY;
			if (x_point > 5)
			{
				if (i % 2 == 0)
				{
					x1 = x1 + (int)stepX * flagX;
				}
				else
				{
					x1 = x1 + ((int)stepX - 1) * flagX;
				}
			}
			else
			{
				if (i % 2 == 0)
				{
					x1 = x1 + (int)stepX * flagY;
				}
				else
				{
					x1 = x1 + ((int)stepX + 1) * flagX;
				}
			}
		}
		printf("i:%d,x1:%d,y1:%d\r\n", i, x1, y1);
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
	// 顺时针移�???
	// 左上顶点
	setServoPwm(pwm_x_left_top_p2, pwm_y_left_top_p2);
	delay_ms(3000);
	TrackX(pwm_x_left_top_p2, pwm_x_right_top_p2, pwm_y_left_top_p2, pwm_y_right_top_p2);
	// 右上顶点
	setServoPwm(pwm_x_right_top_p2, pwm_y_right_top_p2);
	delay_ms(100);
	TrackY(pwm_x_right_top_p2, pwm_x_right_bottom_p2, pwm_y_right_top_p2, pwm_y_right_bottom_p2);
	// 右下顶点
	setServoPwm(pwm_x_right_bottom_p2, pwm_y_right_bottom_p2);
	delay_ms(100);
	TrackX(pwm_x_right_bottom_p2, pwm_x_left_bottom_p2, pwm_y_right_bottom_p2, pwm_y_left_bottom_p2);
	// 左下顶点
	setServoPwm(pwm_x_left_bottom_p2, pwm_y_left_bottom_p2);
	delay_ms(100);
	TrackY(pwm_x_left_bottom_p2, pwm_x_left_top_p2, pwm_y_left_bottom_p2, pwm_y_left_top_p2);
	// 回左上角
	setServoPwm(pwm_x_left_top_p2, pwm_y_left_top_p2);
}
void Problem3(void)
{
	setServoPwm(pwm_x_left_top_p3, pwm_y_left_top_p3);
	delay_ms(1000);
	TrackX(pwm_x_left_top_p3, pwm_x_right_top_p3, pwm_y_left_top_p3, pwm_y_right_top_p3);
	// 右上顶点
	setServoPwm(pwm_x_right_top_p3, pwm_y_right_top_p3);
	delay_ms(100);
	TrackY(pwm_x_right_top_p3, pwm_x_right_bottom_p3, pwm_y_right_top_p3, pwm_y_right_bottom_p3);
	// 右下顶点
	setServoPwm(pwm_x_right_bottom_p3, pwm_y_right_bottom_p3);
	delay_ms(100);
	TrackX(pwm_x_right_bottom_p3, pwm_x_left_bottom_p3, pwm_y_right_bottom_p3, pwm_y_left_bottom_p3);
	// 左下顶点
	setServoPwm(pwm_x_left_bottom_p3, pwm_y_left_bottom_p3);
	delay_ms(100);
	TrackY(pwm_x_left_bottom_p3, pwm_x_left_top_p3, pwm_y_left_bottom_p3, pwm_y_left_top_p3);
	// 回左上角
	setServoPwm(pwm_x_left_top_p3, pwm_y_left_top_p3);
}
void Problem4(void)
{

	int x_left_top_p4 = x_left_top - 1;
	int y_left_top_p4 = y_left_top - 2;
	int x_right_top_p4 = x_right_top - 2;
	int y_right_top_p4 = y_right_top + 3;
	int x_right_bottom_p4 = x_right_bottom - 6;
	int y_right_bottom_p4 = y_right_bottom;
	int x_left_bottom_p4 = x_left_bottom - 3;
	int y_left_bottom_p4 = y_left_bottom - 3;

	rect_orign_x = (int)(x_left_top_p4 + x_right_top_p4 + x_left_bottom_p4 + x_right_bottom_p4) / 4;
	rect_orign_y = (int)(y_left_top_p4 + y_right_top_p4 + y_left_bottom_p4 + y_right_bottom_p4) / 4;
	// printf("rect_orign_x:%d,rect_orign_y:%d\r\n", rect_orign_x, rect_orign_y);
	// printf("x_left_top:%d,y_left_top:%d,x_right_top:%d,y_right_top:%d\r\n", x_left_top_p4, y_left_top_p4, x_right_top_p4, y_right_top_p4);
	// printf("x_left_bottom:%d,y_left_bottom:%d,x_right_bottom:%d,y_right_bottom:%d\r\n", x_left_bottom_p4, y_left_bottom_p4, x_right_bottom_p4, y_right_bottom_p4);

	float pwm_rate_x = 1;
	float pwm_rate_y = 1;

	int x_center_error = origin_x - rect_orign_x;
	int y_center_error = origin_y - rect_orign_y;

	int x_left_top_error = rect_orign_x - x_left_top_p4;
	int y_left_top_error = rect_orign_y - y_left_top_p4;

	int x_right_top_error = rect_orign_x - x_right_top_p4;
	int y_right_top_error = rect_orign_y - y_right_top_p4;

	int x_right_bottom_error = rect_orign_x - x_right_bottom_p4;
	int y_right_bottom_error = rect_orign_y - y_right_bottom_p4;

	int x_left_bottom_error = rect_orign_x - x_left_bottom_p4;
	int y_left_bottom_error = rect_orign_y - y_left_bottom_p4;

	int pwm_x_center;
	int pwm_y_center;
	int pwm_x_left_top;
	int pwm_y_left_top;
	int pwm_x_right_top;
	int pwm_y_right_top;
	int pwm_x_right_bottom;
	int pwm_y_right_bottom;
	int pwm_x_left_bottom;
	int pwm_y_left_bottom;

	pwm_x_center = pwm_orign_x + (int)x_center_error / pwm_rate_x;
	pwm_y_center = pwm_orign_y - (int)y_center_error / pwm_rate_y;

	pwm_x_left_top = pwm_x_center + (int)x_left_top_error / pwm_rate_x;
	pwm_y_left_top = pwm_y_center - (int)(y_left_top_error - 2) / pwm_rate_y;

	pwm_x_right_top = pwm_x_center + (int)(x_right_top_error + 3) / pwm_rate_x;
	pwm_y_right_top = pwm_y_center - (int)y_right_top_error / pwm_rate_y;

	pwm_x_right_bottom = pwm_x_center + (int)x_right_bottom_error / pwm_rate_x;
	pwm_y_right_bottom = pwm_y_center - (int)(y_right_bottom_error + 3) / pwm_rate_y;

	pwm_x_left_bottom = pwm_x_center + (int)(x_left_bottom_error - 2) / pwm_rate_x;
	pwm_y_left_bottom = pwm_y_center - (int)y_left_bottom_error / pwm_rate_y;

	setServoPwm(pwm_x_center, pwm_y_center);
	delay_ms(1000);

	setServoPwm(pwm_x_left_top, pwm_y_left_top);
	// printf("pwm_x_left_top:%d, pwm_y_left_top:%d\r\n", pwm_x_left_top, pwm_y_left_top);
	delay_ms(1000);
	TrackXY(pwm_x_left_top, pwm_x_right_top, pwm_y_left_top, pwm_y_right_top);
	setServoPwm(pwm_x_right_top, pwm_y_right_top);
	printf("pwm_x_right_top:%d, pwm_y_right_top:%d\r\n", pwm_x_right_top, pwm_y_right_top);
	delay_ms(1000);
	TrackXY(pwm_x_right_top, pwm_x_right_bottom, pwm_y_right_top, pwm_y_right_bottom);

	setServoPwm(pwm_x_right_bottom, pwm_y_right_bottom);
	printf("pwm_x_right_bottom:%d, pwm_y_right_bottom:%d\r\n", pwm_x_right_bottom, pwm_y_right_bottom);

	delay_ms(1000);
	TrackXY(pwm_x_right_bottom, pwm_x_left_bottom, pwm_y_right_bottom, pwm_y_left_bottom);

	setServoPwm(pwm_x_left_bottom, pwm_y_left_bottom);
	printf("pwm_x_left_bottom:%d, pwm_y_left_bottom:%d\r\n", pwm_x_left_bottom, pwm_y_left_bottom);

	delay_ms(1000);
	TrackXY(pwm_x_left_bottom, pwm_x_left_top, pwm_y_left_bottom, pwm_y_left_top);

	setServoPwm(pwm_x_left_top, pwm_y_left_top);
	printf("pwm_x_left_top:%d, pwm_y_left_top:%d\r\n", pwm_x_left_top, pwm_y_left_top);

	delay_ms(1000);
}

// 串口三接收中断处理函�?????
uint8_t tempt;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart == &huart3)
	{
		tempt = rxBuffer;
		Openmv_Receive_Data(tempt);
		// 重新启用串口三接收中断，以继续接收数�?????
		HAL_UART_Receive_IT(&huart3, (void *)&rxBuffer, 1);
	}
	else if (huart == &huart1)
	{
		// x �???+ �???-
		// y �???+ �???-
		buffer1[0] = huart->Instance->DR; // Read received data from UART DR register
		if (buffer1[0] == 0x00)
		{
			setServoPwm(675, 783);
			printf("reset\r\n");
		}
		// x�??? +-10
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
		// y�??? +-10
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
		// x�??? +-100
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
		// y�??? +-100
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
		// x�??? +-1
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
		// y�??? +-1
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
		// y�??? +-1
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

void ButtonDebug(int pressedKey)
{
	if (pressedKey)
	{

		switch (model)
		{
		case 1:
			OLED_Clear();
			OLED_ShowString(0, 0, "Main Menu", 8);
			model = 1;
			//				 OLED_ShowNum(94,52,pressedKey,3,12);	//显示ASCII字符的码�??
			switch (pressedKey)
			{
			case 8:
				model = 2;
				if (model > 40)
					model = 1;
				OLED_Clear();
				OLED_ShowString(0, 0, "Model", 1);
				OLED_ShowString(16, 2, "x:", 1);
				OLED_ShowString(16, 4, "y:", 1);
				OLED_ShowNum(32, 0, model, 3, 8); // 显示ASCII字符的码

				break;
			case 16:
				STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t *)load, sizeof(load) / sizeof(uint32_t));
				STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof(uint32_t));
				OLED_Clear();
				OLED_ShowString(0, 0, "save_ok", 1);
				break;
			}
			break;
		case 2:
			switch (pressedKey)
			{
			case 1:
				load[model - 1] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 2:
				load[model - 1]--;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 3:
				load[model - 1] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 9:
				load[model - 2] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 10:
				load[model - 1]++;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 11:
				load[model - 2] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 5:
				load[model - 2]++;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;

			case 7:
				load[model - 2]--;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 16:
				STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t *)load, sizeof(load) / sizeof(uint32_t));
				STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof(uint32_t));
				OLED_Clear();
				OLED_ShowString(0, 0, "save_ok", 1);
				break;
			case 4: // 返回上一级菜�??
				model = 1;
				//						OLED_Clear();
				// 显示回到主菜�??
				break;
				// ... 其他 model1 子菜单按键处�??
			case 8:
				model += 2;
				if (model > 20)
					model = 1;
				OLED_Clear();
				OLED_ShowString(0, 0, "Model", 1);
				OLED_ShowString(16, 2, "x:", 1);
				OLED_ShowString(16, 4, "y:", 1);
				OLED_ShowNum(32, 0, model, 3, 8); // 显示ASCII字符的码

				break;
			}
			break;
		case 4:
			switch (pressedKey)
			{
			case 1:
				load[model - 1] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 2:
				load[model - 1]--;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 3:
				load[model - 1] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 9:
				load[model - 2] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 10:
				load[model - 1]++;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 11:
				load[model - 2] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 5:
				load[model - 2]++;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;

			case 7:
				load[model - 2]--;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 16:
				STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t *)load, sizeof(load) / sizeof(uint32_t));
				STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof(uint32_t));
				OLED_Clear();
				OLED_ShowString(0, 0, "save_ok", 1);
				break;
			case 4: // 返回上一级菜�??
				model = 1;
				//						OLED_Clear();
				// 显示回到主菜�??
				break;
				// ... 其他 model1 子菜单按键处�??
			case 8:
				model += 2;
				if (model > 20)
					model = 1;
				OLED_Clear();
				OLED_ShowString(0, 0, "Model", 1);
				OLED_ShowString(16, 2, "x:", 1);
				OLED_ShowString(16, 4, "y:", 1);
				OLED_ShowNum(32, 0, model, 3, 8); // 显示ASCII字符的码

				break;
			}
			break;
		case 6:
			switch (pressedKey)
			{

			case 1:
				load[model - 1] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 2:
				load[model - 1]--;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 3:
				load[model - 1] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 9:
				load[model - 2] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 10:
				load[model - 1]++;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 11:
				load[model - 2] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 5:
				load[model - 2]++;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;

			case 7:
				load[model - 2]--;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 16:
				STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t *)load, sizeof(load) / sizeof(uint32_t));
				STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof(uint32_t));
				OLED_Clear();
				OLED_ShowString(0, 0, "save_ok", 1);
				break;
			case 4: // 返回上一级菜�??
				model = 1;
				//						OLED_Clear();
				// 显示回到主菜�??
				break;
				// ... 其他 model1 子菜单按键处�??
			case 8:
				model += 2;
				if (model > 20)
					model = 1;
				OLED_Clear();
				OLED_ShowString(0, 0, "Model", 1);
				OLED_ShowString(16, 2, "x:", 1);
				OLED_ShowString(16, 4, "y:", 1);
				OLED_ShowNum(32, 0, model, 3, 8); // 显示ASCII字符的码

				break;
			}
			break;
		case 8:
			switch (pressedKey)
			{

			case 1:
				load[model - 1] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 2:
				load[model - 1]--;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 3:
				load[model - 1] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 9:
				load[model - 2] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 10:
				load[model - 1]++;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 11:
				load[model - 2] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 5:
				load[model - 2]++;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;

			case 7:
				load[model - 2]--;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 16:
				STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t *)load, sizeof(load) / sizeof(uint32_t));
				STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof(uint32_t));
				OLED_Clear();
				OLED_ShowString(0, 0, "save_ok", 1);
				break;
			case 4: // 返回上一级菜�??
				model = 1;
				//						OLED_Clear();
				// 显示回到主菜�??
				break;
				// ... 其他 model1 子菜单按键处�??
			case 8:
				model += 2;
				if (model > 20)
					model = 1;
				OLED_Clear();
				OLED_ShowString(0, 0, "Model", 1);
				OLED_ShowString(16, 2, "x:", 1);
				OLED_ShowString(16, 4, "y:", 1);
				OLED_ShowNum(32, 0, model, 3, 8); // 显示ASCII字符的码

				break;
			}
			break;
		case 10:
			switch (pressedKey)
			{

			case 1:
				load[model - 1] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 2:
				load[model - 1]--;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 3:
				load[model - 1] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 9:
				load[model - 2] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 10:
				load[model - 1]++;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 11:
				load[model - 2] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 5:
				load[model - 2]++;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;

			case 7:
				load[model - 2]--;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 16:
				STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t *)load, sizeof(load) / sizeof(uint32_t));
				STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof(uint32_t));
				OLED_Clear();
				OLED_ShowString(0, 0, "save_ok", 1);
				break;
			case 4: // 返回上一级菜�??
				model = 1;
				//						OLED_Clear();
				// 显示回到主菜�??
				break;
				// ... 其他 model1 子菜单按键处�??
			case 8:
				model += 2;
				if (model > 20)
					model = 1;
				OLED_Clear();
				OLED_ShowString(0, 0, "Model", 1);
				OLED_ShowString(16, 2, "x:", 1);
				OLED_ShowString(16, 4, "y:", 1);
				OLED_ShowNum(32, 0, model, 3, 8); // 显示ASCII字符的码

				break;
			}
			break;
		case 12:
			switch (pressedKey)
			{

			case 1:
				load[model - 1] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 2:
				load[model - 1]--;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 3:
				load[model - 1] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 9:
				load[model - 2] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 10:
				load[model - 1]++;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 11:
				load[model - 2] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 5:
				load[model - 2]++;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;

			case 7:
				load[model - 2]--;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 16:
				STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t *)load, sizeof(load) / sizeof(uint32_t));
				STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof(uint32_t));
				OLED_Clear();
				OLED_ShowString(0, 0, "save_ok", 1);
				break;
			case 4: // 返回上一级菜�??
				model = 1;
				//						OLED_Clear();
				// 显示回到主菜�??
				break;
				// ... 其他 model1 子菜单按键处�??
			case 8:
				model += 2;
				if (model > 20)
					model = 1;
				OLED_Clear();
				OLED_ShowString(0, 0, "Model", 1);
				OLED_ShowString(16, 2, "x:", 1);
				OLED_ShowString(16, 4, "y:", 1);
				OLED_ShowNum(32, 0, model, 3, 8); // 显示ASCII字符的码

				break;
			}
			break;
		case 14:
			switch (pressedKey)
			{

			case 1:
				load[model - 1] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 2:
				load[model - 1]--;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 3:
				load[model - 1] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 9:
				load[model - 2] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 10:
				load[model - 1]++;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 11:
				load[model - 2] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 5:
				load[model - 2]++;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;

			case 7:
				load[model - 2]--;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 16:
				STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t *)load, sizeof(load) / sizeof(uint32_t));
				STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof(uint32_t));
				OLED_Clear();
				OLED_ShowString(0, 0, "save_ok", 1);
				break;
			case 4: // 返回上一级菜�??
				model = 1;
				//						OLED_Clear();
				// 显示回到主菜�??
				break;
				// ... 其他 model1 子菜单按键处�??
			case 8:
				model += 2;
				if (model > 20)
					model = 1;
				OLED_Clear();
				OLED_ShowString(0, 0, "Model", 1);
				OLED_ShowString(16, 2, "x:", 1);
				OLED_ShowString(16, 4, "y:", 1);
				OLED_ShowNum(32, 0, model, 3, 8); // 显示ASCII字符的码

				break;
			}
			break;
		case 16:
			switch (pressedKey)
			{

			case 1:
				load[model - 1] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 2:
				load[model - 1]--;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 3:
				load[model - 1] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 9:
				load[model - 2] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 10:
				load[model - 1]++;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 11:
				load[model - 2] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 5:
				load[model - 2]++;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;

			case 7:
				load[model - 2]--;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 16:
				STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t *)load, sizeof(load) / sizeof(uint32_t));
				STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof(uint32_t));
				OLED_Clear();
				OLED_ShowString(0, 0, "save_ok", 1);
				break;
			case 4: // 返回上一级菜�??
				model = 1;
				//						OLED_Clear();
				// 显示回到主菜�??
				break;
				// ... 其他 model1 子菜单按键处�??
			case 8:
				model += 2;
				if (model > 20)
					model = 1;
				OLED_Clear();
				OLED_ShowString(0, 0, "Model", 1);
				OLED_ShowString(16, 2, "x:", 1);
				OLED_ShowString(16, 4, "y:", 1);
				OLED_ShowNum(32, 0, model, 3, 8); // 显示ASCII字符的码

				break;
			}
			break;
		case 18:
			switch (pressedKey)
			{

			case 1:
				load[model - 1] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 2:
				load[model - 1]--;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 3:
				load[model - 1] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 9:
				load[model - 2] -= 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 10:
				load[model - 1]++;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 11:
				load[model - 2] += 10;
				printf("y:%d\r\n", load[model - 1]);
				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 5:
				load[model - 2]++;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;

			case 7:
				load[model - 2]--;

				OLED_ShowNum(64, 2, load[model - 2], 4, 8); // 显示ASCII字符的码
				OLED_ShowNum(64, 4, load[model - 1], 4, 8); // 显示ASCII字符的码
				break;
			case 16:
				STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t *)load, sizeof(load) / sizeof(uint32_t));
				STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof(uint32_t));
				OLED_Clear();
				OLED_ShowString(0, 0, "save_ok", 1);
				break;
			case 4: // 返回上一级菜�??
				model = 1;
				//						OLED_Clear();
				// 显示回到主菜�??
				break;
				// ... 其他 model1 子菜单按键处�??
			case 8:
				model += 2;
				if (model > 20)
					model = 1;
				OLED_Clear();
				OLED_ShowString(0, 0, "Model", 1);
				OLED_ShowString(16, 2, "x:", 1);
				OLED_ShowString(16, 4, "y:", 1);
				OLED_ShowNum(32, 0, model, 3, 8); // 显示ASCII字符的码

				break;
			}
			break;
		case 20:
			model = 1;
			break;
		}
		if (model > 1)
		{
			setServoPwm(load[model - 2], load[model - 1]);
		}
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
