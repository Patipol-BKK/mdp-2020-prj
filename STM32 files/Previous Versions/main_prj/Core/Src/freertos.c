/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "icm20948.h"
#include "tim.h"
#include "usart.h"
#include "oled.h"
#include <string.h>
#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t motor_ready = 1;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SerialCommTask */
osThreadId_t SerialCommTaskHandle;
const osThreadAttr_t SerialCommTask_attributes = {
  .name = "SerialCommTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime2,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void SerialComm(void *argument);
void StartMotor(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SerialCommTask */
  SerialCommTaskHandle = osThreadNew(SerialComm, NULL, &SerialCommTask_attributes);

  /* creation of MotorTask */
//  MotorTaskHandle = osThreadNew(StartMotor, NULL, &MotorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_SerialComm */
/**
* @brief Function implementing the SerialCommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SerialComm */
void SerialComm(void *argument)
{
  /* USER CODE BEGIN SerialComm */
  /**
   * Data formatting:
   * 	L/S/R - Steering Direction
   * 	XX.X  - Steering Angle (in degrees, not required if moving straight)
   * 	F/W	  - Forward or Backwards
   * 	XXX.X - Distance in cm.
   */
  uint8_t rx_data[12];

  uint8_t top = 0, bot = 0;	// keep track of top and bottom for queue push and pop
  uint8_t BUFFER_LEN = 10;
  uint8_t cmd_buffer[BUFFER_LEN][12];	// command queue with length = 10
  uint8_t i;

//  MX_GPIO_Init();

  uint8_t pwmVal = 1500;

  char* cmd;

  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1); //channel 1 will gnerate the pwm signal
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4); //channel 4 for servo
  /* Infinite loop */
  for(;;)
  {
	HAL_UART_Receive(&huart3, rx_data, 11, 500);
	HAL_UART_Transmit(&huart3, "READY", sizeof("READY"), HAL_MAX_DELAY);
	OLED_ShowString(0, 5, rx_data);
	OLED_Refresh_Gram();
//	strcpy(Rx_data, cmd_buffer[top]);
//	top = (top + 1)%BUFFER_LEN;
//	rx_data[0] = 'L';
	switch((int)(rx_data[0]))
	{
		case 'L':
			htim1.Instance ->CCR4 = 90;
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal);
			osDelay(1000);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
			osDelay(200);
			break;
		case 'R':
			htim1.Instance ->CCR4 = 220;
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal);
			osDelay(1000);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
			osDelay(200);
			break;
		case 'F':
			htim1.Instance ->CCR4 = 145;
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal);
			osDelay(1000);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
			osDelay(200);
			break;
		case 'B':
			htim1.Instance ->CCR4 = 145;
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal);
			osDelay(1000);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
			osDelay(200);
			break;
	}

//	if(motor_ready)
//	{
////		MotorTaskHandle = osThreadNew(StartMotor, (void*)"L40.3F001.0", &MotorTask_attributes);
//
//		strcpy(cmd, rx_data);
//		switch(cmd[0])
//		{
//		case 'L':
//
//		}
//
//	}
//	OLED_ShowString(0, 5, Rx_data);
//	OLED_Refresh_Gram();
	osDelay(1);
  }
  /* USER CODE END SerialComm */
}

/* USER CODE BEGIN Header_StartMotor */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotor */
void StartMotor(void *argument)
{
  /* USER CODE BEGIN StartMotor */
	if(argument == NULL)
	{
		return;
	}
	char const *cmd = (char const*)argument;
	switch(cmd[0])
	{

	}
	OLED_ShowString(0, 5, cmd);
	OLED_Refresh_Gram();
  /* USER CODE END StartMotor */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

