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
  .priority = (osPriority_t) osPriorityNormal,
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
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Motor_R */
osThreadId_t Motor_RHandle;
const osThreadAttr_t Motor_R_attributes = {
  .name = "Motor_R",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
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
void StartDefaultTask(void *argument);
void Display(void *argument);
void LeftMotor(void *argument);
void RightMotor(void *argument);
void EncoderL(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t pwmVal = 2000, pwmVal_S = 1000*12/28, pwmVal_L = 1000;
uint8_t Buffer[5];
char flagL;
char flagR;
char flagNext;
int flagfin;

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
  Buffer[1] = 'W';
  Buffer[2] = '0';
  Buffer[3] = '0';
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
  /* USER CODE BEGIN 2 */
  OLED_Init();
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

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(EncoderL, NULL, &EncoderTask_attributes);

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
  sConfig.IC2Filter = 0;
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
  flagfin = 0;
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
	sprintf(hello, "flag %d", flagfin);
	OLED_ShowString(10,10,hello);
	OLED_Refresh_Gram();
    osDelay(1);
    if(flagfin >= 2){
//    	osDelay(300);
		HAL_UART_Transmit(&huart3, "R", sizeof("R"), HAL_MAX_DELAY);
//    	osDelay(300);

//    	osDelay(500);
		flagfin = 0;
	}
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
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  int cnt1, cnt2, pulse;
  uint32_t tick, dist, angle, pulseneeded, pulsetotal;
  uint8_t hello[20];

  cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
  tick = HAL_GetTick();

  int value;
  flagL = '1';
  /* Infinite loop */
  for(;;){
	  while(flagfin > 0)osDelay(1);
	  value = (Buffer[2] - '0')*100 + (Buffer[3] - '0')*10 + Buffer[4] - '0';
	  dist = value;
	  angle = value;
	  if(Buffer[0] == 'F'){
		  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	  }
	  else if(Buffer[0] == 'B'){
		  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	  }
	  else{
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		  continue;
	  }
	  switch(Buffer[1]){
	  case 'L':
		  htim1.Instance ->CCR4 = 91;
		  osDelay(300);
		  pulseneeded = angle*34*pwmVal_S/pwmVal_L;
//		  pulseneeded = angle*0.115*59;
		  //pulseneeded = angle*0.35*59*60/90;
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal_S);
		  break;
	  case 'R':
		  htim1.Instance ->CCR4 = 240;
		  osDelay(300);
		  pulseneeded = angle*39.4;
//		  pulseneeded = angle*0.408*59;
		  //pulseneeded = angle*0.35*59*115/90;
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal_L);

		  break;
	  default:
		  htim1.Instance ->CCR4 = 147;
		  osDelay(300);
		  pulseneeded = dist*45.4;
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal);
		  break;
	  }
	  pulsetotal = 0;
	  do{
		  if(HAL_GetTick()-tick > 50L){
				sprintf(hello, "%cpulseL:%5d\0", flagL, pulseneeded);
				OLED_ShowString(10,30,hello);

				cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
				if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
					if(cnt2<cnt1)
					{
						pulse = cnt1 - cnt2;
						//sprintf(hello, "total1:%5d\0", pulsetotal);
					}
					else
					{
						pulse = (65535 - cnt2) + cnt1;
						//sprintf(hello, "total2:%5d\0", pulsetotal);
					}
				}
				else{
					if(cnt2>cnt1)
					{
						pulse = cnt2 - cnt1;
						//sprintf(hello, "total3:%5d\0", pulsetotal);
					}
					else
					{
						pulse = (65535 - cnt1) + cnt2;
						//sprintf(hello, "total4:%5d\0", pulsetotal);
					}
				}
				pulsetotal = (pulsetotal + pulse)%65535;
				//OLED_ShowString(10,40,hello);
				cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
				tick = HAL_GetTick();
		  }
	  }while(pulsetotal < pulseneeded);
	  flagfin++;
	  Buffer[0] = 'd';
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
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
  /* USER CODE BEGIN LeftMotor */
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  int cnt1, cnt2, pulse;
  uint32_t tick, dist, angle, pulseneeded, pulsetotal;
  uint8_t hello[20];

  cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
  tick = HAL_GetTick();

  flagR = '1';
  flagfin = 0;
  int value;
  /* Infinite loop */
  for(;;){
	  while(flagfin > 0)osDelay(1);
	  value = (Buffer[2] - '0')*100 + (Buffer[3] - '0')*10 + Buffer[4] - '0';
	  dist = value;
	  angle = value;
	  if(Buffer[0] == 'F'){
		  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
	  }
	  else if(Buffer[0] == 'B'){
		  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	  }
	  else{
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		  continue;
	  }
	  switch(Buffer[1]){
	  case 'L':
//		  htim1.Instance ->CCR4 = 90;
		  osDelay(300);
		  pulseneeded = angle*34;
//		  pulseneeded = angle*0.408*59;
		  //pulseneeded = angle*0.35*59*60/90;
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal_L);
		  break;
	  case 'R':
//		  htim1.Instance ->CCR4 = 212;
		  osDelay(300);
		  pulseneeded = angle*39.4*pwmVal_S/pwmVal_L;
//		  pulseneeded = angle*0.115*59;
		  //pulseneeded = angle*0.35*59*115/90;
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal_S);
		  break;
	  default:
//		  htim1.Instance ->CCR4 = 147;
		  osDelay(300);
		  pulseneeded = dist*45.4;
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal);
		  break;
	  }
	  pulsetotal = 0;
	  do{
		  if(HAL_GetTick()-tick > 50L){
				sprintf(hello, "%cpulseR:%5d\0", flagR, pulseneeded);
				OLED_ShowString(10,40,hello);

				cnt2 = __HAL_TIM_GET_COUNTER(&htim3);
				if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)){
					if(cnt2<cnt1)
					{
						pulse = cnt1 - cnt2;
						//sprintf(hello, "total1:%5d\0", pulsetotal);
					}
					else
					{
						pulse = (65535 - cnt2) + cnt1;
						//sprintf(hello, "total2:%5d\0", pulsetotal);
					}
				}
				else{
					if(cnt2>cnt1)
					{
						pulse = cnt2 - cnt1;
						//sprintf(hello, "total3:%5d\0", pulsetotal);
					}
					else
					{
						pulse = (65535 - cnt1) + cnt2;
						//sprintf(hello, "total4:%5d\0", pulsetotal);
					}
				}
				pulsetotal = (pulsetotal + pulse)%65535;
				//OLED_ShowString(10,40,hello);
				cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
				tick = HAL_GetTick();
		  }
	  }while(pulsetotal < pulseneeded);
	  flagfin++;
	  Buffer[0] = 'd';
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
//	  osDelay(500);
	  //HAL_UART_Transmit(&huart3, "R", sizeof("R"), HAL_MAX_DELAY);

  }
  /* USER CODE END LeftMotor */
}

/* USER CODE BEGIN Header_EncoderL */
/**
* @brief Function implementing the EncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EncoderL */
void EncoderL(void *argument)
{
  /* USER CODE BEGIN EncoderL */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END EncoderL */
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

