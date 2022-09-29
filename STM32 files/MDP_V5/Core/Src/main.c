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

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Motor_L */
osThreadId_t Motor_LHandle;
const osThreadAttr_t Motor_L_attributes = {
  .name = "Motor_L",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Motor_R */
osThreadId_t Motor_RHandle;
const osThreadAttr_t Motor_R_attributes = {
  .name = "Motor_R",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GyroReadTask */
osThreadId_t GyroReadTaskHandle;
const osThreadAttr_t GyroReadTask_attributes = {
  .name = "GyroReadTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void Display(void *argument);
void LeftMotor(void *argument);
void RightMotor(void *argument);
void GyroFunc(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t pwmVal = 1500, pwmVal_S = 2000*12/28, pwmVal_L = 2000;
uint8_t Buffer[5];
int32_t heading_rbt = 0;
double heading_f_rbt = 0;
ICM20948 imu;

PID_TypeDef TPID;
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

  Buffer[0] = 'F';
  Buffer[1] = 'L';
  Buffer[2] = '0';
  Buffer[3] = '9';
  Buffer[4] = '0';

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
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

  /* creation of Motor_R */
  Motor_RHandle = osThreadNew(RightMotor, NULL, &Motor_R_attributes);

  /* creation of GyroReadTask */
  GyroReadTaskHandle = osThreadNew(GyroFunc, NULL, &GyroReadTask_attributes);

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

}

/* USER CODE BEGIN 4 */
uint8_t is_right(int32_t head1, int32_t head2)
{
	if(head2 > head1 && head2 - head1 < 18000) return 1;
	else if(head2 < head1 && head2 + 36000 - head1 < 18000)return 1;
	return 0;
}
int32_t get_heading_error(int32_t head1, int32_t head2, uint8_t is_counterclock)
{
	if(is_counterclock){
		if(head1 < head2){
			return head2 - head1;
		}
		else{

		}
	}
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
  HAL_UART_Receive_IT(&huart3,(uint8_t *) Buffer,5);
  for(;;)
  {
	HAL_UART_Receive_IT(&huart3,(uint8_t *) Buffer,5);
	sprintf(hello, "buff:%s", Buffer);
	OLED_ShowString(10,20,hello);
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
	sprintf(hello, "V5");
	OLED_ShowString(10,10,hello);
	sprintf(hello, "%d", (int)heading_f_rbt); // @suppress("Float formatting support")
	OLED_ShowString(10,30,hello);
	OLED_Refresh_Gram();
    osDelay(1);
  }
  /* USER CODE END Display */
}

/* USER CODE BEGIN Header_LeftMotor */
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
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  int cnt1, cnt2, pulse;
  int32_t tick, dist, pulseneeded, pulsetotal;
  uint8_t hello[20];

  int32_t current_heading = heading_rbt;
  int32_t target_heading, start_heading, heading_error;
  int32_t angle_progress, delta_angle, prev_heading;

  cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
  tick = HAL_GetTick();
  int8_t value;


  uint8_t* status = IMU_Initialise(&imu, &hi2c1, &huart3);
  HAL_Delay(1000);
  Gyro_calibrateHeading(&imu);

  double current_angle = 0;
  double PID_out;
  double target_angle = 90;

  char sbuf[10];

  PID(&TPID, &current_angle, &PID_out, &target_angle, 1, 2, 3, _PID_P_ON_E, _PID_CD_DIRECT);

  PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&TPID, 10);
  PID_SetOutputLimits(&TPID, -100, 100);

  /* Infinite loop */
  for(;;){
	  do{
		  osDelay(1);
	  }while(Buffer[0] == 'd');
	  value = (Buffer[2] - '0')*100 + (Buffer[3] - '0')*10 + Buffer[4] - '0';
//	  dist = value;


	  if(Buffer[0]=='F' && Buffer[1]=='L'){
		  target_angle = -((double)value);
		  current_angle = 0;
		  htim1.Instance ->CCR4 = 91;
		  for(;;){
			  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

			  taskENTER_CRITICAL();
			  IMU_GyroReadHeading(&imu);
			  taskEXIT_CRITICAL();
			  current_angle = current_angle + imu.gyro[2];
			  PID_Compute(&TPID);

//			  sprintf(sbuf, "%d ", (int)current_angle);
//			  HAL_UART_Transmit(&huart3, (uint8_t *)sbuf, sizeof(sbuf), HAL_MAX_DELAY);
			  sprintf(sbuf, "%d,%d", (int)PID_out,(int)current_angle);
			  HAL_UART_Transmit(&huart3, (uint8_t *)sbuf, sizeof(sbuf), HAL_MAX_DELAY);
			  HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);

			  if(PID_out < 0){
				  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

				  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,(double)pwmVal_S*(-PID_out/100.0f));
				  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,(double)pwmVal_L*(-PID_out/100.0f));
			  }
			  else{
				  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);

				  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,(double)pwmVal_S*(PID_out/100.0f));
				  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,(double)pwmVal_L*(PID_out/100.0f));
			  }
			  osDelayUntil(10);
		  }


	  }
//	  if(Buffer[0] == 'F'){
//		  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
//	  }
//	  else if(Buffer[0] == 'B'){
//		  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
//	  }
//	  else{
//		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
//		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
//		  continue;
//	  }
//	  switch(Buffer[1]){
//	  case 'L':
//		  htim1.Instance ->CCR4 = 91;
//		  HAL_Delay(500);
////		  pulseneeded = angle*34*pwmVal_S/pwmVal_L;
////		  pulseneeded = angle*0.115*59;
//		  //pulseneeded = angle*0.35*59*60/90;
////		  current_heading = heading_rbt;
////		  start_heading = current_heading;
////		  if(Buffer[0] == 'F'){
////			  if(current_heading < angle*100) target_heading = current_heading + 36000 - angle*100;
////			  else target_heading = current_heading - angle*100;
////			  angle_progress = 0;
////		  }
////		  else if(Buffer[0] == 'B'){
////			  target_heading = current_heading + angle*100;
////			  if(target_heading >= 36000) target_heading = target_heading - 36000;
////		  }
////		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal_S);
////		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal_L);
//		  break;
//	  case 'R':
//		  htim1.Instance ->CCR4 = 240;
//		  HAL_Delay(500);
////		  pulseneeded = angle*39.4;
////		  pulseneeded = angle*0.408*59;
//		  //pulseneeded = angle*0.35*59*115/90;
////		  current_heading = heading_rbt;
////		  start_heading = current_heading;
////		  if(Buffer[0] == 'F'){
////			  target_heading = current_heading + angle*100;
////			  if(target_heading >= 36000) target_heading = target_heading - 36000;
////		  }
////		  else if(Buffer[0] == 'B'){
////			  if(current_heading < angle*100) target_heading = current_heading + 36000 - angle*100;
////			  else target_heading = current_heading - angle*100;
////		  }
////		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal_L);
////		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal_S);
//		  break;
//	  default:
//		  htim1.Instance ->CCR4 = 148.4;
//		  HAL_Delay(500);
//		  //pulseneeded = dist*68*1.03;
//		  pulseneeded = dist*45.4*10/6.55*70/66;
//		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal);
//		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal);
//		  pulsetotal = 0;
//		  do{
//			  if(HAL_GetTick()-tick > 50L){
//					sprintf(hello, "pulseL:%5d\0", pulseneeded);
//					OLED_ShowString(10,30,hello);
//
//					cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
//					if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
//						if(cnt2<cnt1)
//						{
//							pulse = cnt1 - cnt2;
//							//sprintf(hello, "total1:%5d\0", pulsetotal);
//						}
//						else
//						{
//							pulse = (65535 - cnt2) + cnt1;
//							//sprintf(hello, "total2:%5d\0", pulsetotal);
//						}
//					}
//					else{
//						if(cnt2>cnt1)
//						{
//							pulse = cnt2 - cnt1;
//							//sprintf(hello, "total3:%5d\0", pulsetotal);
//						}
//						else
//						{
//							pulse = (65535 - cnt1) + cnt2;
//							//sprintf(hello, "total4:%5d\0", pulsetotal);
//						}
//					}
//					pulsetotal = (pulsetotal + pulse)%65535;
//					//OLED_ShowString(10,40,hello);
//					cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
//					tick = HAL_GetTick();
//			  }
//		  }while(pulsetotal < pulseneeded);
//		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
//		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
//		  Buffer[0] = 'd';
//		  HAL_UART_Transmit(&huart3,"R", sizeof("R"), HAL_MAX_DELAY);
//		  continue;
//	  }
//	  delta_angle = 0;
//	  angle_progress = 0;
//	  prev_heading = heading_rbt;
//	  if(Buffer[0] == 'F' && Buffer[1] == 'L'){
//		  do{
//			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal_S*1.2);
//			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal_L*1.2);
//
//			  delta_angle = prev_heading - heading_rbt;
//			  prev_heading = heading_rbt;
//			  if(delta_angle < -18000) delta_angle = delta_angle + 36000;
//			  angle_progress = angle_progress + delta_angle;
//			  sprintf(hello, "currentL%d ", angle_progress);
//			  OLED_ShowString(10,40,hello);
//			  sprintf(hello, "targetL%d ", angle);
//			  OLED_Refresh_Gram();
//			  OLED_ShowString(10,50,hello);
//			  osDelay(1);
//		  }while(angle_progress < angle);
//		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
//		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
//
//		  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
//
//		  do{
//			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal_S*0.7);
//			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal_L*0.7);
//
//			  delta_angle = heading_rbt - prev_heading;
//			  prev_heading = heading_rbt;
//			  if(delta_angle < -18000) delta_angle = delta_angle + 36000;
//			  angle_progress = angle_progress - delta_angle;
//			  sprintf(hello, "currentS%d ", angle_progress);
//			  OLED_ShowString(10,40,hello);
//			  sprintf(hello, "targetS%d ", angle);
//			  OLED_ShowString(10,50,hello);
//			  OLED_Refresh_Gram();
//			  osDelay(1);
//		  }while(angle_progress > angle);
//	  }
//	  else if(Buffer[0] == 'B' && Buffer[1] == 'L'){
//		  heading_error = target_heading - current_heading;
//		  if(heading_error < -18000) heading_error = heading_error + 36000;
//		  do{
//			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal_S*1.2);
//			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal_L*1.2);
//
//			  delta_angle = heading_rbt - prev_heading;
//			  prev_heading = heading_rbt;
//			  if(delta_angle < -18000) delta_angle = delta_angle + 36000;
//			  angle_progress = angle_progress - delta_angle;
//			  sprintf(hello, "currentL%d ", angle_progress);
//			  OLED_ShowString(10,40,hello);
//			  sprintf(hello, "targetL%d ", angle);
//			  OLED_ShowString(10,50,hello);
//			  OLED_Refresh_Gram();
//			  osDelay(1);
//		  }while(angle_progress > angle);
//		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
//		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
//
//		  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
//
//		  do{
//			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal_S*1.2);
//			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal_L*1.2);
//
//			  delta_angle = prev_heading - heading_rbt;
//			  prev_heading = heading_rbt;
//			  if(delta_angle < -18000) delta_angle = delta_angle + 36000;
//			  angle_progress = angle_progress + delta_angle;
//			  sprintf(hello, "currentL%d ", angle_progress);
//			  OLED_ShowString(10,40,hello);
//			  sprintf(hello, "targetL%d ", angle);
//			  OLED_ShowString(10,50,hello);
//			  OLED_Refresh_Gram();
//			  osDelay(1);
//		  }while(angle_progress < angle);
////		  while(!is_right(target_heading, current_heading)){
////			  current_heading = heading_rbt;
////			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal_S*0.5);
////			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal_L*0.5);
////			  HAL_Delay(1);
////		  }
//	  }
//	  if(Buffer[0] == 'F' && Buffer[1] == 'R'){
//		  while(abs(current_heading - target_heading ) > 100){
//			  current_heading = heading_rbt;
//			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal_S*1);
//			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal_L*1);
//			  HAL_Delay(1);
//		  }
////		  while(is_right(target_heading, current_heading)){
////			  current_heading = heading_rbt;
////			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal_S*0.5);
////			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal_L*0.5);
////			  HAL_Delay(1);
////		  }
//	  }
//	  else if(Buffer[0] == 'B' && Buffer[1] == 'R'){
//		  while(abs(current_heading - target_heading ) > 100){
//			  current_heading = heading_rbt;
//			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal_S*1);
//			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal_L*1);
//			  HAL_Delay(1);
//		  }
////		  while(!is_right(target_heading, current_heading)){
////			  current_heading = heading_rbt;
////			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal_S*0.5);
////			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal_L*0.5);
////			  HAL_Delay(1);
////		  }
//	  }
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
	  Buffer[0] = 'd';
	  HAL_UART_Transmit(&huart3,"R", sizeof("R"), HAL_MAX_DELAY);
  }
  /* USER CODE END LeftMotor */
}

/* USER CODE BEGIN Header_RightMotor */
/**
* @brief Function implementing the Motor_R thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RightMotor */
void RightMotor(void *argument)
{
  /* USER CODE BEGIN RightMotor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RightMotor */
}

/* USER CODE BEGIN Header_GyroFunc */
/**
* @brief Function implementing the GyroReadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GyroFunc */
void GyroFunc(void *argument)
{
  /* USER CODE BEGIN GyroFunc */



	  char sbuf[6];
	  uint16_t gyro_val, left, right;


	  while (1)
	  {
//		  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
//
//		  taskENTER_CRITICAL();
//		  heading_f_rbt = IMU_GyroReadHeading(&imu);
//		  taskEXIT_CRITICAL();
//		  current_angle = current_angle + imu.gyro[2];
//		  PID_Compute(&TPID);
//
//		  sprintf(sbuf, "%5.2lf ", current_angle);
//		  HAL_UART_Transmit(&huart3, (uint8_t *)sbuf, sizeof(sbuf), HAL_MAX_DELAY);
//		  sprintf(sbuf, "%5.2lf ", PID_out);
//		  HAL_UART_Transmit(&huart3, (uint8_t *)sbuf, sizeof(sbuf), HAL_MAX_DELAY);
//		  HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);



		  osDelayUntil(10);
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

