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
#include "xunji.h"
#include "mpu6050.h"
#include "delay.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "pid.h"
#include "car.h"
#include "oled.h"
#include "math.h"
#include "stmflash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BUFFER_SIZE 4
#define FLASH_SAVE_ADDR  0X08010000
// 定义行和列的数量
#define NUM_ROWS 4
#define NUM_COLS 4

// 行和列对应的IO口
GPIO_TypeDef* row_ports[NUM_ROWS] = {GPIOA, GPIOC, GPIOA, GPIOA};
uint16_t row_pins[NUM_ROWS] = {GPIO_PIN_2, GPIO_PIN_2, GPIO_PIN_4, GPIO_PIN_7};

GPIO_TypeDef* col_ports[NUM_COLS] = {GPIOC, GPIOB, GPIOF, GPIOF};
uint16_t col_pins[NUM_COLS] = {GPIO_PIN_5, GPIO_PIN_1, GPIO_PIN_11, GPIO_PIN_7};
// 编码器有关变量
	uint32_t encoderCount = 0;  // 计数器
	uint32_t encoderSpeed = 0;  // 速度
	uint32_t enc1_prev = 0;     // 上次计数器的值

	float target_angle=0.0;
	int flag=0;
	GPIO_PinState ledStates[7];
	float current_yaw;
	
	// 缓冲区用于存储接收到的数据
	uint8_t rxBuffer[BUFFER_SIZE];
	uint32_t rxIndex = 0;
	uint8_t buffer[3];
	uint8_t buffer_index = 0;
	uint8_t buffer1[1];
	u32 load[20];
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
uint16_t ADC_value; //AD值
float Real_value; //真实值

int16_t receivedX;
int16_t receivedY;
int keyNumbers[NUM_ROWS][NUM_COLS] = {
    {1, 2, 3, 4},
    {5, 6, 7, 8},
    {9, 10, 11, 12},
    {13, 14, 15, 16}
};
/* USER CODE END PV */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
u8 datatemp[1];
int flag_servo;
int angle_x1=90;
int angle_y1=75;
	

uint16_t pulse_x =1332;
uint16_t pulse_y =2124;

int center_x=0;
int center_y=0;

int pencil_lefttop_x=0;
int pencil_lefttop_y=0;
int pencil_righttop_x=0;
int pencil_righttop_y=0;
int pencil_leftbottom_x=0;
int pencil_leftbottom_y=0;
int pencil_rightbottom_x=0;
int pencil_rightbottom_y=0;
	
int rect_lefttop_x=0;
int rect_lefttop_y=0;
int rect_righttop_x=0;
int rect_righttop_y=0;
int rect_leftbottom_x=0;
int rect_leftbottom_y=0;
int rect_rightbottom_x=0;
int rect_rightbottom_y=0;

// 定义PID常数
#define Kp_x 1
#define Ki_x 0.0
#define Kd_x 0.01

#define Kp_y 0.2
#define Ki_y 0.0
#define Kd_y 0.01

// 初始化PID变量
float prev_error_x = 0;
float integral_x = 0;
float derivative_x = 0;

float prev_error_y = 0;
float integral_y = 0;
float derivative_y = 0;
#define DEBOUNCE_DELAY_MS 20 // 调整此值以适应您的需求
int isButtonPressed(GPIO_TypeDef* port, uint16_t pin) {
    uint32_t startTime = HAL_GetTick(); // 获取当前时间

    while (HAL_GetTick() - startTime < DEBOUNCE_DELAY_MS) {
        if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) {
            // 按键仍然按下
            HAL_Delay(10); // 延迟一小段时间，确保按键稳定
        } else {
            // 按键已经释放
            return 0;
        }
    }

    // 在一段时间内保持按下状态，认为按键按下
    return 1;
}
//舵机pid
void setServoPosition(float angle_x,float angle_y) {
	// 舵机
	 pulse_x = ((angle_x * 2500) / 270)+499;
	 pulse_y = ((angle_y * 1000) / 120)+1499;

	// 定时器3
	TIM3->CCR1 = pulse_x;
	TIM3->CCR2 = pulse_y;
	printf("angx: %d angy: %d\r\n",angle_x1,angle_y1);
	printf("pwmx: %d  pwmy: %d\r\n",pulse_x,pulse_y);
	printf("\r\n");
}
void implement()
{
	int ccrx,ccry;
	
	if(flag_servo)
	{
		
		ccrx = PID_Level(receivedX);
		ccry = PID_vertical(receivedY);
		
		pulse_x = pulse_x - ccrx;
		pulse_y = pulse_y - ccry;
		
		if(pulse_x>=3000)pulse_x=3000;
		else if(pulse_x<=500) pulse_x=500;
		if(pulse_y>=2500)pulse_y=2500;
		else if(pulse_y<=1500) pulse_y=1500;
		
		//        // 更新舵机位置    
				TIM3->CCR1 = pulse_x;
				TIM3->CCR2 = pulse_y;			
		printf("pidX%d  pidY%d\r\n", pulse_x, pulse_y);
		flag_servo = 0;
		
	}
}

////获取编码器信息
//uint32_t getEncoderSpeed(void) {
//	uint32_t enc1 = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim4));
//	uint32_t pulseChange = enc1 - enc1_prev;
//	uint32_t speed = pulseChange * 10;
//	enc1_prev = enc1;
//	return speed;
//}
//AD数模转换
//uint32_t ADC_Value;
//void getVoltage(void) {
//	HAL_ADC_Start(&hadc1);
//	HAL_ADC_PollForConversion(&hadc1, 50);

//	if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)) {
//		ADC_Value = HAL_ADC_GetValue(&hadc1);

//		printf("ADC1 Reading : %d \r\n", ADC_Value);
//		printf("PA4 True Voltage value : %.4f \r\n", ADC_Value * 3.3f / 4096);

//	}
//	HAL_Delay(1000);
//}
//调试用
//void car_wait(void) {

//	car_stop();
//	delay_ms(50);
//	motor_forward();
//}
int currentLevel = 0;
int previousLevel = 0;
int currentMenu = 0; // 当前所在菜单
int model=1;
int scanKeyMatrix(void) {
    int pressedKey = 0; // 默认值表示没有按键按下
    
    for (int i = 0; i < NUM_COLS; i++) {
        // 设置当前列为低电平
        HAL_GPIO_WritePin(col_ports[i], col_pins[i], GPIO_PIN_RESET);
        delay_ms(30);
        // 检查每一行的状态
        for (int j = 0; j < NUM_ROWS; j++) {
            if (HAL_GPIO_ReadPin(row_ports[j], row_pins[j]) == GPIO_PIN_RESET) {
                pressedKey = keyNumbers[j][i]; // 获取按键编号
            }
        }
        
        // 恢复当前列为高电平
        HAL_GPIO_WritePin(col_ports[i], col_pins[i], GPIO_PIN_SET);
    }
    
    return pressedKey;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
//	HAL_TIM_Base_Start(&htim1);
//	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim1);       //通过这行代码，以中断的方式启动定时器。  
	
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
//	setServoPosition(angle_x1,angle_y1);
	STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof (uint32_t));
//  pencil_lefttop_x=STMFLASH_ReadWord(FLASH_SAVE_ADDR);
//  pencil_lefttop_y=STMFLASH_ReadWord(FLASH_SAVE_ADDR+4);
//  pencil_righttop_x=STMFLASH_ReadWord(FLASH_SAVE_ADDR+8);
//  pencil_righttop_y=STMFLASH_ReadWord(FLASH_SAVE_ADDR+12);
//  pencil_leftbottom_x=STMFLASH_ReadWord(FLASH_SAVE_ADDR+16);
//  pencil_leftbottom_y=STMFLASH_ReadWord(FLASH_SAVE_ADDR+20);
//  pencil_rightbottom_x=STMFLASH_ReadWord(FLASH_SAVE_ADDR+24);
//  pencil_rightbottom_y=STMFLASH_ReadWord(FLASH_SAVE_ADDR+28);
//	
//  rect_lefttop_x=STMFLASH_ReadWord(FLASH_SAVE_ADDR+32);
//  rect_lefttop_y=STMFLASH_ReadWord(FLASH_SAVE_ADDR+36);
//  rect_righttop_x=STMFLASH_ReadWord(FLASH_SAVE_ADDR+40);
//  rect_righttop_y=STMFLASH_ReadWord(FLASH_SAVE_ADDR+44);
//  rect_leftbottom_x=STMFLASH_ReadWord(FLASH_SAVE_ADDR+48);
//  rect_leftbottom_y=STMFLASH_ReadWord(FLASH_SAVE_ADDR+52);
//  rect_rightbottom_x=STMFLASH_ReadWord(FLASH_SAVE_ADDR+56);
//  rect_rightbottom_y=STMFLASH_ReadWord(FLASH_SAVE_ADDR+60);
//	
//	center_x=STMFLASH_ReadWord(FLASH_SAVE_ADDR+64);
//	center_y=STMFLASH_ReadWord(FLASH_SAVE_ADDR+68);
 
	printf("%d %d\r\n",load[0],load[1]); 
	// 使能串口三接收中断
//    HAL_UART_Receive_IT(&huart3, &rxBuffer[rxIndex], 1);
		HAL_UART_Receive_IT(&huart1, &buffer1[1], 1);
		PID_Init();
// 重新启用串口三接收中断，以继续接收数据
//        HAL_UART_Receive_IT(&huart2, (uint8_t *)buffer, 1);
//		Mpu6050_Init();
	OLED_Init();
	OLED_Clear();
//		OLED_ShowString(0,0,"Hello",8);
//		OLED_ShowCHinese(0,2,0);  //物
//		OLED_ShowCHinese(16,2,1);	//联
//		OLED_ShowCHinese(32,2,2);	//网
//		OLED_ShowCHinese(48,2,3);	//小
//		OLED_ShowCHinese(64,2,4);	//白
//		OLED_ShowString(80,2,"Jayce",16);
//OLED_ShowString(0,0,"key number:",8);
						
				 
	//HAL_ADC_Start_IT(&hadc1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1) {

	int pressedKey = scanKeyMatrix();
	if (pressedKey) {
		switch (model) {
			case 1:
						OLED_Clear();
						OLED_ShowString(0, 0, "Main Menu", 8);
						model = 1;
//				 OLED_ShowNum(94,52,pressedKey,3,12);	//显示ASCII字符的码值
				switch (pressedKey) {					
					case 8:
						model=2;
						if (model > 40)
							model = 1;
						OLED_Clear();
						OLED_ShowString(0, 0, "Model", 1);
						OLED_ShowString(16, 2, "x:", 1);
						OLED_ShowString(16, 4, "y:", 1);
						OLED_ShowNum(32, 0, model, 3, 8);	//显示ASCII字符的码
						
						break;
				}
				break;
			case 2:
				switch (pressedKey) {
					
					case 2:							
						load[model-1]--;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 10:								
						load[model-1]++;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 5:							
						load[model-2]++;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 7:							
						load[model-2]--;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					case 16:
						STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t*)load, sizeof(load) / sizeof (uint32_t));
						STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof (uint32_t));
						printf("x:%d\r\n", load[0]);
						printf("y:%d\r\n", load[1]);
						break;
					case 4: // 返回上一级菜单
						model = 1;
//						OLED_Clear();
						// 显示回到主菜单
						break;
						// ... 其他 model1 子菜单按键处理
					case 8:
						model+=2;
						if (model > 20)
							model = 1;
						OLED_Clear();
						OLED_ShowString(0, 0, "Model", 1);
						OLED_ShowString(16, 2, "x:", 1);
						OLED_ShowString(16, 4, "y:", 1);
						OLED_ShowNum(32, 0, model, 3, 8);	//显示ASCII字符的码
						
						break;
				}
				break;
				
				case 4:
				switch (pressedKey) {	
					case 2:							
						load[model-1]--;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 10:								
						load[model-1]++;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 5:							
						load[model-2]++;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 7:							
						load[model-2]--;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					case 16:
						STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t*)load, sizeof(load) / sizeof (uint32_t));
						STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof (uint32_t));
						printf("x:%d\r\n", load[0]);
						printf("y:%d\r\n", load[1]);
						break;
					case 4: // 返回上一级菜单
						model = 1;
//						OLED_Clear();
						// 显示回到主菜单
						break;
						// ... 其他 model1 子菜单按键处理
					case 8:
						model+=2;
						if (model > 20)
							model = 1;
						OLED_Clear();
						OLED_ShowString(0, 0, "Model", 1);
						OLED_ShowString(16, 2, "x:", 1);
						OLED_ShowString(16, 4, "y:", 1);
						OLED_ShowNum(32, 0, model, 3, 8);	//显示ASCII字符的码
						
						break;
				}
				break;
				case 6:
				switch (pressedKey) {
					
					case 2:							
						load[model-1]--;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 10:								
						load[model-1]++;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 5:							
						load[model-2]++;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 7:							
						load[model-2]--;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					case 16:
						STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t*)load, sizeof(load) / sizeof (uint32_t));
						STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof (uint32_t));
						printf("x:%d\r\n", load[0]);
						printf("y:%d\r\n", load[1]);
						break;
					case 4: // 返回上一级菜单
						model = 1;
//						OLED_Clear();
						// 显示回到主菜单
						break;
						// ... 其他 model1 子菜单按键处理
					case 8:
						model+=2;
						if (model > 20)
							model = 1;
						OLED_Clear();
						OLED_ShowString(0, 0, "Model", 1);
						OLED_ShowString(16, 2, "x:", 1);
						OLED_ShowString(16, 4, "y:", 1);
						OLED_ShowNum(32, 0, model, 3, 8);	//显示ASCII字符的码
						
						break;
				}
				break;
				case 8:
				switch (pressedKey) {
					
					case 2:							
						load[model-1]--;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 10:								
						load[model-1]++;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 5:							
						load[model-2]++;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 7:							
						load[model-2]--;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					case 16:
						STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t*)load, sizeof(load) / sizeof (uint32_t));
						STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof (uint32_t));
						printf("x:%d\r\n", load[0]);
						printf("y:%d\r\n", load[1]);
						break;
					case 4: // 返回上一级菜单
						model = 1;
//						OLED_Clear();
						// 显示回到主菜单
						break;
						// ... 其他 model1 子菜单按键处理
					case 8:
						model+=2;
						if (model > 20)
							model = 1;
						OLED_Clear();
						OLED_ShowString(0, 0, "Model", 1);
						OLED_ShowString(16, 2, "x:", 1);
						OLED_ShowString(16, 4, "y:", 1);
						OLED_ShowNum(32, 0, model, 3, 8);	//显示ASCII字符的码
						
						break;
				}
				break;
				case 10:
				switch (pressedKey) {
					
					case 2:							
						load[model-1]--;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 10:								
						load[model-1]++;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 5:							
						load[model-2]++;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 7:							
						load[model-2]--;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					case 16:
						STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t*)load, sizeof(load) / sizeof (uint32_t));
						STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof (uint32_t));
						printf("x:%d\r\n", load[0]);
						printf("y:%d\r\n", load[1]);
						break;
					case 4: // 返回上一级菜单
						model = 1;
//						OLED_Clear();
						// 显示回到主菜单
						break;
						// ... 其他 model1 子菜单按键处理
					case 8:
						model+=2;
						if (model > 20)
							model = 1;
						OLED_Clear();
						OLED_ShowString(0, 0, "Model", 1);
						OLED_ShowString(16, 2, "x:", 1);
						OLED_ShowString(16, 4, "y:", 1);
						OLED_ShowNum(32, 0, model, 3, 8);	//显示ASCII字符的码
						
						break;
				}
				break;
				case 12:
				switch (pressedKey) {
					
					case 2:							
						load[model-1]--;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 10:								
						load[model-1]++;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 5:							
						load[model-2]++;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 7:							
						load[model-2]--;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					case 16:
						STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t*)load, sizeof(load) / sizeof (uint32_t));
						STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof (uint32_t));
						printf("x:%d\r\n", load[0]);
						printf("y:%d\r\n", load[1]);
						break;
					case 4: // 返回上一级菜单
						model = 1;
//						OLED_Clear();
						// 显示回到主菜单
						break;
						// ... 其他 model1 子菜单按键处理
					case 8:
						model+=2;
						if (model > 20)
							model = 1;
						OLED_Clear();
						OLED_ShowString(0, 0, "Model", 1);
						OLED_ShowString(16, 2, "x:", 1);
						OLED_ShowString(16, 4, "y:", 1);
						OLED_ShowNum(32, 0, model, 3, 8);	//显示ASCII字符的码
						
						break;
				}
				break;
				case 14:
				switch (pressedKey) {
					
					case 2:							
						load[model-1]--;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 10:								
						load[model-1]++;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 5:							
						load[model-2]++;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 7:							
						load[model-2]--;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					case 16:
						STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t*)load, sizeof(load) / sizeof (uint32_t));
						STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof (uint32_t));
						printf("x:%d\r\n", load[0]);
						printf("y:%d\r\n", load[1]);
						break;
					case 4: // 返回上一级菜单
						model = 1;
//						OLED_Clear();
						// 显示回到主菜单
						break;
						// ... 其他 model1 子菜单按键处理
					case 8:
						model+=2;
						if (model > 20)
							model = 1;
						OLED_Clear();
						OLED_ShowString(0, 0, "Model", 1);
						OLED_ShowString(16, 2, "x:", 1);
						OLED_ShowString(16, 4, "y:", 1);
						OLED_ShowNum(32, 0, model, 3, 8);	//显示ASCII字符的码
						
						break;
				}
				break;
				case 16:
				switch (pressedKey) {
					
					case 2:							
						load[model-1]--;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 10:								
						load[model-1]++;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 5:							
						load[model-2]++;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 7:							
						load[model-2]--;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					case 16:
						STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t*)load, sizeof(load) / sizeof (uint32_t));
						STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof (uint32_t));
						printf("x:%d\r\n", load[0]);
						printf("y:%d\r\n", load[1]);
						break;
					case 4: // 返回上一级菜单
						model = 1;
//						OLED_Clear();
						// 显示回到主菜单
						break;
						// ... 其他 model1 子菜单按键处理
					case 8:
						model+=2;
						if (model > 20)
							model = 1;
						OLED_Clear();
						OLED_ShowString(0, 0, "Model", 1);
						OLED_ShowString(16, 2, "x:", 1);
						OLED_ShowString(16, 4, "y:", 1);
						OLED_ShowNum(32, 0, model, 3, 8);	//显示ASCII字符的码
						
						break;
				}
				break;
				case 18:
				switch (pressedKey) {
					
					case 2:							
						load[model-1]--;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 10:								
						load[model-1]++;
					printf("y:%d\r\n",load[model-1]);
					OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 5:							
						load[model-2]++;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					
					case 7:							
						load[model-2]--;
					printf("x:%d\r\n",angle_x1);
						OLED_ShowNum(64, 2, load[model-2], 4, 8);	//显示ASCII字符的码
						OLED_ShowNum(64, 4, load[model-1], 4, 8);	//显示ASCII字符的码
						break;
					case 16:
						STMFLASH_Write(FLASH_SAVE_ADDR, (uint32_t*)load, sizeof(load) / sizeof (uint32_t));
						STMFLASH_Read(FLASH_SAVE_ADDR, load, sizeof(load) / sizeof (uint32_t));
						printf("x:%d\r\n", load[0]);
						printf("y:%d\r\n", load[1]);
						break;
					case 4: // 返回上一级菜单
						model = 1;
//						OLED_Clear();
						// 显示回到主菜单
						break;
						// ... 其他 model1 子菜单按键处理
					case 8:
						model+=2;
						if (model > 20)
							model = 1;
						OLED_Clear();
						OLED_ShowString(0, 0, "Model", 1);
						OLED_ShowString(16, 2, "x:", 1);
						OLED_ShowString(16, 4, "y:", 1);
						OLED_ShowNum(32, 0, model, 3, 8);	//显示ASCII字符的码
						
						break;
				}
				break;
				case 20:
					model=1;
				break;
				
		}
	}
}
//		if (flag_servo){
//		servo_pid(receivedX,receivedY);
//			flag_servo=0;
//		}
//    flag=readLEDsState(ledStates);
//		 current_yaw=get_yaw();
////		printf("%f\r\n",current_yaw);
////		//    OLED_ShowString(0,0,"gjkbhk",8);
//////				OLED_DrawBMP(40, 2, 88, 8);
//		if(flag!=7)
//		 track(readLEDsState(ledStates),500);	//巡线
//		 CarStraight(0.0);
//		else{
//			target_angle-=75.0f;
//			delay_ms(500);
//			car_wait();		   
//    while(!CarRight90(target_angle)) 
//			;		
//		car_wait();
//		flag=0;
//	}
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//    if (htim->Instance == TIM1) {
//        implement();
//			 HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
//    }
//}
void Mpu6050_Init(void) {
	printf("%s\r\n", "MPU Init...");
	while (MPU_Init())
		; // 鍒濆鍖朚PU6050
	while (mpu_dmp_init()) {

		printf("%s\r\n", "Mpu6050 Init Wrong!");
	}

	printf("%s\r\n", "Mpu6050 Init OK!");
}

// Function to update PID parameters via serial communication
void update_pid_parameters(uint8_t* buffer,PID_Controller *pid) {    
    float new_kp=buffer[0], new_ki=buffer[1], new_kd=buffer[2];	
    // Assuming you have a function to read data from the serial port and store it in 'buffer'
    // For example, if you are using UART:
    // read_from_uart(buffer);

    // Parse the received data to extract new parameter values
    
//	printf("%f,%f,%f\r\n",new_kp,new_ki,new_kd);

    // Update PID parameters if new values are valid
    if (new_kp > 0 && new_ki >= 0 && new_kd >= 0) {
        pid->Kp = new_kp;
        pid->Ki = new_ki;
        pid->Kd = new_kd;
    }
		printf("pid:%f,%f,%f\r\n",pid->Kp,pid->Ki,pid->Kd);
}

// 串口三接收中断处理函数

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        // 读取接收到的数据，并存储到缓冲区中
        rxBuffer[rxIndex++] = huart3.Instance->DR;

        // 判断是否接收完成
        if (rxIndex >= BUFFER_SIZE)
        {
            // 接收完成，解析X坐标和Y坐标
             receivedX = (int16_t)((rxBuffer[1] << 8) | rxBuffer[0]);
             receivedY = (int16_t)((rxBuffer[3] << 8) | rxBuffer[2]);
							printf("%d  %d\r\n",receivedX,receivedY);
            // 使用 receivedX 和 receivedY 进行后续处理
            // 重置缓冲区索引，准备下一次接收
					  flag_servo=1;
            rxIndex = 0;
						implement();
//					servo_pidControl();
        }
				// 重新启用串口三接收中断，以继续接收数据
        HAL_UART_Receive_IT(&huart3, &rxBuffer[rxIndex], 1);       
    }
		if (huart == &huart2)
    {
			buffer[buffer_index] = huart->Instance->DR; // Read received data from UART DR register
//			printf("%d\r\n",buffer[buffer_index]);
			buffer_index++;
			if (buffer_index >= 3) {
				update_pid_parameters(buffer,&pid_track_left);
				update_pid_parameters(buffer,&pid_track_right);
            buffer_index=0;
        }
//			printf("pid:%f,%f,%f\r\n",pid_track_left.Kp,pid_track_left.Ki,pid_track_left.Kd);
			 // 重新启用串口三接收中断，以继续接收数据
        HAL_UART_Receive_IT(&huart2, (uint8_t *)buffer, 1);
			}	
		if (huart == &huart1)
    {
			buffer1[0] = huart->Instance->DR; // Read received data from UART DR register
			if(buffer1[0]==1){
				angle_x1+=1;		
			}
			if(buffer1[0]==2){
				angle_x1-=1;			
			}
			if(buffer1[0]==3){				
			angle_y1+=1;
			}
			if(buffer1[0]==4){				
			angle_y1-=1;
			}
			if (angle_x1>=270)
				angle_x1=270;
			if (angle_y1>=120)
				angle_y1=120;
			if (angle_x1<=0)
				angle_x1=0;
			if(angle_y1<=0)
				angle_y1=0;
			 
			load[0] = angle_x1;
			load[1]=angle_y1;
			STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)load,sizeof(load)/sizeof (uint32_t));
			STMFLASH_Read(FLASH_SAVE_ADDR,load,sizeof(load)/sizeof (uint32_t));
			printf("x:%d\r\n", load[0]);
      printf("y:%d\r\n", load[1]);
			
//			printf("data:%d\r\n", datatemp[0]);
				// 重新启用串口接收中断，以继续接收数据		
//				setServoPosition(angle_x1,angle_y1);
			
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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
