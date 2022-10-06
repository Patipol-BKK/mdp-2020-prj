/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "ICM20948.h"
#include "math.h"
#include "pid.h"
#include "stdio.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Motor_L */
osThreadId_t Motor_LHandle;
const osThreadAttr_t Motor_L_attributes = {
  .name = "Motor_L",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GyroTask */
osThreadId_t GyroTaskHandle;
const osThreadAttr_t GyroTask_attributes = {
  .name = "GyroTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void Display(void *argument);
void LeftMotor(void *argument);
void GyroFunc(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t pwmVal = 2300, pwmVal_S = 2000*12/28, pwmVal_L = 2000;
double min_pwm_ratio = 0.3, max_pwm_dif = 0.5;
uint8_t Buffer[5];
int32_t heading_rbt = 0;
int32_t t_heading = 0;
double current_angle = 0, current_gyro = 0;

uint8_t is_calibrated = 0;
double straight_error = 0.0;
double error_x = 0.0, error_y = 0.0;

double travel_dist = 0;
double prev_dist = 0;
int32_t encoder_position = 0;

double relative_x = 0;
double relative_y = 0;
ICM20948 imu;

PID_TypeDef Turning_PID, Straight_PID, StraightErr_PID;
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

  Buffer[0] = 'd';
  Buffer[1] = 'W';
  Buffer[2] = '0';
  Buffer[3] = '9';
  Buffer[4] = 'd';

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(Display, NULL, &DisplayTask_attributes);

  /* creation of Motor_L */
  Motor_LHandle = osThreadNew(LeftMotor, NULL, &Motor_L_attributes);

  /* creation of GyroTask */
  GyroTaskHandle = osThreadNew(GyroFunc, NULL, &GyroTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_Pin */
  GPIO_InitStruct.Pin = SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t is_right(int32_t head1, int32_t head2)
{
	if(head2 > head1 && head2 - head1 < 18000) return 1;
	else if(head2 < head1 && head2 + 36000 - head1 < 18000)return 1;
	return 0;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint8_t hello[20];
  /* Infinite loop */
//  HAL_UART_Receive_IT(&huart3,(uint8_t *) Buffer,5);
  for(;;)
  {

//	sprintf(hello, "buff:%s", Buffer);
//	OLED_ShowString(10,20,hello);
	HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Display */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Display */
void Display(void *argument)
{
  /* USER CODE BEGIN Display */
  uint8_t hello[20] = "testV5!\0";
  /* Infinite loop */
  for(;;)
  {
	sprintf(hello, "V5.1");
	OLED_ShowString(10,10,hello);
	OLED_Refresh_Gram();
    osDelay(1);
  }
  /* USER CODE END Display */
}

/* USER CODE BEGIN Header_LeftMotor */
double PID_out;
double target_angle = 90;
double target_x = 0, target_y = 0;
/**
* @brief Function implementing the Motor_L thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LeftMotor */
void LeftMotor(void *argument)
{
  /* USER CODE BEGIN LeftMotor */

  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);


  uint8_t hello[20];



  int16_t value;


  while(!is_calibrated){
	  osDelay(100);
  }
  htim1.Instance ->CCR4 = 148.4;




  uint8_t target_is_before = 0;

  double left_pwm = 0, right_pwm = 0;
  double PID_dist;

  double target_dist = 0;
  double slip_x = 0, slip_y = 0;

  char sbuf[10];
  t_heading = current_angle;
  HAL_UART_Receive_DMA (&huart3, Buffer, 5);
  double TURNING_RAD_L = 22 - 7.95,  TURNING_RAD_R = 23 + 7.95;
  double local_x = 0, local_y = 0, local_r = 0, local_theta = 0;
  /* Infinite loop */
  for(;;){
	  // Loop until next command is received
	  do{
		  osDelay(100);
	  }while(Buffer[4] == 'd');
	  target_x = 0;
	  target_y = 0;
	  local_x = 0;
	  local_y = 0;
	  local_theta = 0;
	  local_r = 0;
	  // Convert instruction value to int
	  value = (Buffer[2] - '0')*100 + (Buffer[3] - '0')*10 + Buffer[4] - '0';
//	  dist = value;
	  if((Buffer[0]=='F' && Buffer[1]=='L')){
		  local_x = -TURNING_RAD_L + TURNING_RAD_L*cos((double)value*(M_PI / 180.0));
		  local_y = TURNING_RAD_L*sin((double)value*(M_PI / 180.0));

		  local_theta = (180.0 - (double)value)/2.0 - 90.0;
	  }
	  else if((Buffer[0]=='B' && Buffer[1]=='L')){
		  local_x = -TURNING_RAD_L + TURNING_RAD_L*cos((double)value*(M_PI / 180.0));
		  local_y = -TURNING_RAD_L*sin((double)value*(M_PI / 180.0));

		  local_theta = -90.0 - (180.0 - (double)value)/2.0;
	  }
	  else if((Buffer[0]=='F' && Buffer[1]=='R')){
		  local_x = TURNING_RAD_R - TURNING_RAD_R*cos((double)value*(M_PI / 180.0));
		  local_y = TURNING_RAD_R*sin((double)value*(M_PI / 180.0));

		  local_theta = 90.0 - (180.0 - (double)value)/2.0;
	  }
	  else if((Buffer[0]=='B' && Buffer[1]=='R')){
		  local_x = TURNING_RAD_R - TURNING_RAD_R*cos((double)value*(M_PI / 180.0));
		  local_y = -TURNING_RAD_R*sin((double)value*(M_PI / 180.0));

		  local_theta = 90.0 + (180.0 - (double)value)/2.0;
	  }

	  local_r = sqrt(local_x*local_x + local_y*local_y);
	  target_x = local_r * sin(((double)t_heading + local_theta)*(M_PI / 180.0));
	  target_y = local_r * cos(((double)t_heading + local_theta)*(M_PI / 180.0));

	  // For counter-clockwise turning
	  if((Buffer[0]=='F' && Buffer[1]=='L')||(Buffer[0]=='B' && Buffer[1]=='R')){
		  t_heading = t_heading - value;	// Set target heading
		  target_angle = (double)t_heading;
		  target_is_before = 1;
	  }
	  // For clockwise turning
	  else if ((Buffer[0]=='F' && Buffer[1]=='R')||(Buffer[0]=='B' && Buffer[1]=='L')){
		  t_heading = t_heading + value;
		  target_angle = (double)t_heading;
		  target_is_before = 0;
	  }
	  else if(Buffer[1] == 'W'){
		  target_angle = (double)t_heading;
	  }

	  // Set servo values
	  if(Buffer[1] == 'L'){
		  htim1.Instance ->CCR4 = 91;
	  }
	  else if(Buffer[1] == 'R'){
		  htim1.Instance ->CCR4 = 240;
	  }
	  else{
		  htim1.Instance ->CCR4 = 148.4;
	  }
	  // Wait for servo to turn
	  osDelay(550);
	  // If currently running turning instruction
	  if(Buffer[1] != 'W'){
		  PID_out = 0;
		  // Set PID Controller (constants are Kp,Ki,Kd)
		  PID(&Turning_PID, &current_angle, &PID_out, &target_angle, 0.021, 0.1, 0.0, _PID_P_ON_E, _PID_CD_DIRECT);

		  PID_SetMode(&Turning_PID, _PID_MODE_AUTOMATIC);
		  PID_SetSampleTime(&Turning_PID, 10);
		  PID_SetOutputLimits(&Turning_PID, -1.0f+min_pwm_ratio, 1.0f-min_pwm_ratio);

		  // For debugging
	  //	  sprintf(sbuf, "%d ", (int)target_angle);
	  //	  HAL_UART_Transmit(&huart3, (uint8_t *)sbuf, sizeof(sbuf), HAL_MAX_DELAY);
	  //	  sprintf(sbuf, "%d", (int)(-1.0f*(double)target_is_before)*(target_angle - current_angle));
	  //	  HAL_UART_Transmit(&huart3, (uint8_t *)sbuf, sizeof(sbuf), HAL_MAX_DELAY);
	  //	  HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);

		  relative_x = 0;
		  relative_y = 0;

		  // Loop until robot's heading passes target heading
		  while(2*(0.5f - (double)target_is_before)*(target_angle - current_angle)>0){
			  // Blinking LED for checking program crashes
			  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

			  // Compute next pwm val
			  PID_Compute(&Turning_PID);

//			  taskENTER_CRITICAL();
			  // If steering left
			  if(Buffer[1] == 'L')
			  {
				  // Forward
				  if(PID_out < 0){
					  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

					  // Set motor speed to minimum power +/- PID output
					  if(2*(0.5f - (double)target_is_before)*(target_angle - current_angle) < 10){
						  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,(double)pwmVal_S*min_pwm_ratio);
						  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,(double)pwmVal_L*min_pwm_ratio);
					  }
					  else{
						  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,(double)pwmVal_S*(-PID_out+min_pwm_ratio));
						  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,(double)pwmVal_L*(-PID_out+min_pwm_ratio));
					  }
				  }
				  // Backwards
				  else{
					  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);

					  // Set motor speed to minimum power +/- PID output
					  if(2*(0.5f - (double)target_is_before)*(target_angle - current_angle) < 10){
						  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,(double)pwmVal_S*min_pwm_ratio);
						  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,(double)pwmVal_L*min_pwm_ratio);
					  }
					  else{
						  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,(double)pwmVal_S*(PID_out+min_pwm_ratio));
						  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,(double)pwmVal_L*(PID_out+min_pwm_ratio));
					  }
				  }
			  }
			  // If steering right
			  else if(Buffer[1] == 'R')
			  {
				  // Backwards
				  if(PID_out < 0){
					  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);

					  // Set motor speed to minimum power +/- PID output
					  if(2*(0.5f - (double)target_is_before)*(target_angle - current_angle) < 10){
						  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,(double)pwmVal_L*min_pwm_ratio);
						  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,(double)pwmVal_S*min_pwm_ratio);
					  }
					  else{
						  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,(double)pwmVal_L*(-PID_out+min_pwm_ratio));
						  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,(double)pwmVal_S*(-PID_out+min_pwm_ratio));
					  }
				  }
				  // Forwards
				  else{
					  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

					  // Set motor speed to minimum power +/- PID output
					  if(2*(0.5f - (double)target_is_before)*(target_angle - current_angle) < 10){
						  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,(double)pwmVal_L*min_pwm_ratio);
						  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,(double)pwmVal_S*min_pwm_ratio);
					  }
					  else{
						  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,(double)pwmVal_L*(PID_out+min_pwm_ratio));
						  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,(double)pwmVal_S*(PID_out+min_pwm_ratio));
					  }
				  }
			  }
//			  taskEXIT_CRITICAL();

			  // Loops every 10ms
			  osDelayUntil(pdMS_TO_TICKS(10));
		  }
		  // Set both motor's speed to 0
		  taskENTER_CRITICAL();
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		  taskEXIT_CRITICAL();
	  }
	  // If currently running straight line instruction
	  else{
		  // Reset traveled distance to 0
		  travel_dist = 0;
		  encoder_position = 0;
		  if(Buffer[0] == 'F'){
//			  target_dist = (double)value * 100.0/102.5;
			  target_dist = (double)value * 100.0/102.5;
		  }
		  else if(Buffer[0] == 'B'){
//			  target_dist = -(double)value * 100.0/102.5;
			  target_dist = -(double)value * 100.0/102.5;
		  }
		  if(((int)target_angle)%360 == 90 || ((int)target_angle)%360+360 == 90){
			  target_dist = target_dist + error_x;
			  error_x = 0;
			  target_x = target_dist;
			  target_y = 0;
		  }
		  else if(((int)target_angle)%360 == 270 || ((int)target_angle)%360+360 == 270){
			  target_dist = target_dist - error_x;
			  error_x = 0;
			  target_x = -target_dist;
			  target_y = 0;
		  }
		  else if(((int)target_angle)%360 == 0 || ((int)target_angle)%360+360 == 0){
			  target_dist = target_dist + error_y;
			  error_y = 0;
			  target_x = 0;
			  target_y = target_dist;
		  }
		  else if(((int)target_angle)%360 == 180 || ((int)target_angle)%360+360 == 180){
			  target_dist = target_dist - error_y;
			  error_y = 0;
			  target_x = 0;
			  target_y = -target_dist;
		  }
		  if(target_dist > 0){
			  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
			  target_is_before = 1;
		  }
		  if(target_dist < 0){
			  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
			  target_is_before = 0;
		  }
		  PID_dist = 0;
		  PID_out = 0;
		  // Set straight distance PID controller (constants are Kp,Ki,Kd)
		  PID(&Straight_PID, &travel_dist, &PID_dist, &target_dist, 0.02, 0.07, 0.0, _PID_P_ON_E, _PID_CD_DIRECT);

		  PID_SetMode(&Straight_PID, _PID_MODE_AUTOMATIC);
		  PID_SetSampleTime(&Straight_PID, 10);
		  PID_SetOutputLimits(&Straight_PID, -1.0f+min_pwm_ratio, 1.0f-min_pwm_ratio);

		  // Set straight line error correction PID controller (constants are Kp,Ki,Kd)
		  PID(&StraightErr_PID, &current_angle, &PID_out, &target_angle, 0.05, 0.02, 0.0, _PID_P_ON_E, _PID_CD_DIRECT);

		  PID_SetMode(&StraightErr_PID, _PID_MODE_AUTOMATIC);
		  PID_SetSampleTime(&StraightErr_PID, 10);
		  PID_SetOutputLimits(&StraightErr_PID, -max_pwm_dif, max_pwm_dif);

		  // Start motor speed to 0
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,(double)0);
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,(double)0);
		  relative_x = 0;
		  relative_y = 0;

		  // Loop until traveled distance exceeds target distance (target_is_before is to account for backward movements)
		  while((2*((double)target_is_before - 0.5f)*(target_dist - travel_dist)>0)){

			  // For Debugging
//			  sprintf(sbuf, "%7d ", (int)current_pulse);
//			  HAL_UART_Transmit(&huart3, (uint8_t *)sbuf, 8, HAL_MAX_DELAY);
//			  sprintf(sbuf, "%7d", (int)target_pulse);
//			  HAL_UART_Transmit(&huart3, (uint8_t *)sbuf, 8, HAL_MAX_DELAY);
//			  HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);

			  // Comput PID values
			  PID_Compute(&Straight_PID);
			  PID_Compute(&StraightErr_PID);
			  if(Buffer[0] == 'F')
			  {
				  // Change pwm ratio for both motors to correct if heading deviates from straight line
				  htim1.Instance ->CCR4 = 148.4 + (target_angle - current_angle)*3;
				  left_pwm = (double)pwmVal*(1+PID_out);
				  right_pwm = (double)pwmVal*(1-PID_out);
//				  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,(double)pwmVal*(1+PID_out));
//				  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,(double)pwmVal*(1-PID_out));
			  }
			  else if(Buffer[0] == 'B')
			  {
				  // Change pwm ratio for both motors to correct if heading deviates from straight line
				  htim1.Instance ->CCR4 = 148.4 - (target_angle - current_angle)*3;
				  left_pwm = (double)pwmVal*(1-PID_out);
				  right_pwm = (double)pwmVal*(1+PID_out);
//				  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,(double)pwmVal*(1-PID_out));
//				  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,(double)pwmVal*(1+PID_out));
			  }

//			  taskENTER_CRITICAL();
			  if(PID_dist < 0){
				  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);

				  if(2*((double)target_is_before - 0.5f)*(target_dist - travel_dist)<10){
					  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, left_pwm*min_pwm_ratio);
					  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2, right_pwm*min_pwm_ratio);
				  }
				  else{
					  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, left_pwm*(min_pwm_ratio-PID_dist));
					  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2, right_pwm*(min_pwm_ratio-PID_dist));
				  }

			  }
			  else{
				  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

				  if(2*((double)target_is_before - 0.5f)*(target_dist - travel_dist)<10){
					  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, left_pwm*min_pwm_ratio);
					  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2, right_pwm*min_pwm_ratio);
				  }
				  else{
					  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, left_pwm*(min_pwm_ratio+PID_dist));
					  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2, right_pwm*(min_pwm_ratio+PID_dist));
				  }
			  }
//			  taskEXIT_CRITICAL();
			  osDelayUntil(pdMS_TO_TICKS(10));
		  }
	  }
	  error_x = error_x + (relative_x - target_x);
	  error_y = error_y + (relative_y - target_y);

	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);		// Sets both wheel to 0 speed
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
	  Buffer[4] = 'd';		// Sets start of instruction buffer to invalid command so the stm doesn't repeat itself
	  HAL_UART_Transmit(&huart3,"R", sizeof("R"), HAL_MAX_DELAY);	// Sends ready signal to RPi
  }
  /* USER CODE END LeftMotor */
}

/* USER CODE BEGIN Header_GyroFunc */
/**
* @brief Function implementing the GyroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GyroFunc */
void GyroFunc(void *argument)
{
  /* USER CODE BEGIN GyroFunc */
  uint8_t* status = IMU_Initialise(&imu, &hi2c1, &huart3);	// Initialize gyro
  uint8_t dispBuff[20];

  // Calibrate gyroscope
  taskENTER_CRITICAL();
  sprintf(dispBuff, "Calibr Gyro..");	// Prints current heading angle (x1000)
  OLED_ShowString(10,30,dispBuff);
//  OLED_Refresh_Gram();
  osDelay(2000);
  Gyro_calibrateHeading(&imu, pdMS_TO_TICKS(21));	// Sample gyro data every 21ms for 1024 samples and use as offset
  osDelay(2000);
  taskEXIT_CRITICAL();
  is_calibrated = 1;	// Set finish calibration flag to start running other task
  char sbuf[10];

  int32_t encoder_prev = -1, encoder_cur = -1, dif = 0;
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  straight_error = 0.0;

  uint32_t prevTick = 0;
  uint32_t elapsedMs, tick;


  /* Infinite loop */
  for(;;)
  {
	  // Read gyro value
	  taskENTER_CRITICAL();
	  IMU_GyroReadHeading(&imu);
	  taskEXIT_CRITICAL();

	  // Update heading values
	  current_gyro = current_gyro + imu.gyro[2];		// Manual gyro offset
	  current_angle = current_gyro*1.0f;		// Increase if robot turns too much

	  // Check if it's the first time running
	  taskENTER_CRITICAL();
	  if(encoder_prev == -1 || encoder_cur == -1){
		  encoder_cur = __HAL_TIM_GET_COUNTER(&htim2);
		  encoder_prev = encoder_cur;
		  prevTick = xTaskGetTickCount();
	  }
	  else{
		  encoder_prev = encoder_cur;
		  encoder_cur = __HAL_TIM_GET_COUNTER(&htim2);
		  // Deal with the encoder value wrapping around at 65535
		  if(encoder_cur >= encoder_prev){
			  if(encoder_cur - encoder_prev < 32768){
				  dif = encoder_prev - encoder_cur;
			  }
			  else{
				  dif = encoder_prev + (65535 - encoder_cur);
			  }
		  }
		  else{
			  if(encoder_prev - encoder_cur < 32768){
				  dif = encoder_prev - encoder_cur;
			  }
			  else{
				  dif = encoder_prev - (65535 + encoder_cur);
			  }
		  }
//		  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
//			  if(encoder_cur <= encoder_prev){
//				  dif = encoder_prev - encoder_cur;
//			  }
//			  else{
//				  dif = encoder_prev + (65535 - encoder_cur);
//			  }
//		  }
//		  else{
//			  if(encoder_cur >= encoder_prev){
//				  dif = encoder_prev - encoder_cur;
//			  }
//			  else{
//				  dif = encoder_prev - (65535 + encoder_cur);
//			  }
//		  }
		  tick = xTaskGetTickCount();
		  elapsedMs = tick-prevTick;
		  prevTick = tick;

		  relative_x = relative_x + dif*sin(current_angle*(M_PI / 180.0))*0.01308996939;
		  relative_y = relative_y + dif*cos(current_angle*(M_PI / 180.0))*0.01308996939;

		  encoder_position = encoder_position + dif;
		  straight_error = straight_error + dif*sin((current_angle - target_angle)*(M_PI / 180.0));
	  }
//	  travel_dist = (double)encoder_position * 0.01310615989;		// Edit constant to calibrate straight line distance
	  travel_dist = (double)encoder_position * 0.01308996939;
	  taskEXIT_CRITICAL();
	  sprintf(dispBuff, "X=%5d,Y=%5d", (int)target_x, (int)target_y);	// Prints current heading angle (x1000)
	  OLED_ShowString(10,30,dispBuff);
	  sprintf(dispBuff, "dif=%5d", dif);	// Prints current heading angle (x1000)
	  OLED_ShowString(10,20,dispBuff);
	  sprintf(dispBuff, "X=%5d,Y=%5d", (int)encoder_cur, (int)error_y);	// Prints current heading angle (x1000)
	  OLED_ShowString(10,40,dispBuff);
	  sprintf(dispBuff, "angle=%5d ", (int)current_angle*1000);	// Prints current heading angle (x1000)
	  OLED_ShowString(10,10,dispBuff);
	  sprintf(dispBuff, "X=%5d,Y=%5d", (int)relative_x, (int)relative_y);	// Prints current heading angle (x1000)
	  OLED_ShowString(10,50,dispBuff);

	  if(!HAL_GPIO_ReadPin(SW_GPIO_Port,SW_Pin)){
		  encoder_position = 0;
		  travel_dist = 0;
		  encoder_prev = -1;
		  encoder_cur = -1;
		  dif = 0;
		  current_gyro = 0;
		  current_angle = 0;
		  t_heading = 0;
		  relative_x = 0;
		  relative_y = 0;
		  error_x = 0;
		  error_y = 0;
		  taskENTER_CRITICAL();
		  sprintf(dispBuff, "Calibr Gyro..");	// Prints current heading angle (x1000)
		  OLED_ShowString(10,30,dispBuff);
		  osDelay(2000);
		  Gyro_calibrateHeading(&imu, pdMS_TO_TICKS(21));	// Sample gyro data every 21ms for 1024 samples and use as offset
		  osDelay(2000);
		  taskEXIT_CRITICAL();
	  }
	  // For debugging
//  	  sprintf(sbuf, "%7d ", (int)(encoder_position));
//  	  HAL_UART_Transmit(&huart3, (uint8_t *)sbuf, 8, HAL_MAX_DELAY);
//  	  sprintf(sbuf, "%7d ", (int)(encoder_cur));
//  	  HAL_UART_Transmit(&huart3, (uint8_t *)sbuf, 8, HAL_MAX_DELAY);

//	  sprintf(sbuf, "%7.2f ", imu.gyro[2]);
//	  HAL_UART_Transmit(&huart3, (uint8_t *)sbuf, 8, HAL_MAX_DELAY);
//	  sprintf(sbuf, "%7d ", (int)((imu.gyro[2])*1000));
//	  HAL_UART_Transmit(&huart3, (uint8_t *)sbuf, 8, HAL_MAX_DELAY);
////	  sprintf(sbuf, "%9ul", (DWT->CYCCNT));
////	  HAL_UART_Transmit(&huart3, (uint8_t *)sbuf, 9, HAL_MAX_DELAY);
//	  sprintf(sbuf, "\r\n");
//	  HAL_UART_Transmit(&huart3, (uint8_t *)sbuf, 2, HAL_MAX_DELAY);

      osDelayUntil(pdMS_TO_TICKS(10));
  }
  /* USER CODE END GyroFunc */
}

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

