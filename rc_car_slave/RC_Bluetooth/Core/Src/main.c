/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union {
    struct {
        uint8_t b;
        uint8_t r;
        uint8_t g;
    } color;
    uint32_t data;
} PixelRGB_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_DISTANCE 400 // HC-SR04의 유효 최대 거리 (cm)
#define MIN_DISTANCE 30 // 멈추는 거리

#define NOTE_F_freq  262 // 부저음 - 앞 c4
#define NOTE_B_freq  440    //뒤 A4
#define NOTE Duration 500


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId UltrasonicTaskHandle;
osThreadId BluetoothTaskHandle;
osThreadId MotorTaskHandle;
osThreadId MultiTaskHandle;
osMessageQId motorQueueHandle;
/* USER CODE BEGIN PV */

// --- 전역 변수 ---
uint8_t rx3_data;
uint8_t rx2_data;
volatile char g_rx3_command = 'f';

volatile int g_final_dist_forward = 9999;
volatile int g_final_dist_back = 9999;
int obstacle_detected = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
void StartUltrasonicTask(void const * argument);
void StartBluetoothTask(void const * argument);
void StartMotorTask(void const * argument);
void StartMultiTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROT0TYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE {
	if (ch == '\n')
		HAL_UART_Transmit(&huart2, (uint8_t*) "\r", 1, 0xFFFF);
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);

	return ch;
}

void smartcar_forward() {
	HAL_GPIO_WritePin(RR_F_GPIO_Port, RR_F_Pin, 1);
	HAL_GPIO_WritePin(RR_B_GPIO_Port, RR_B_Pin, 0);
	HAL_GPIO_WritePin(RF_F_GPIO_Port, RF_F_Pin, 1);
	HAL_GPIO_WritePin(RF_B_GPIO_Port, RF_B_Pin, 0);
	HAL_GPIO_WritePin(LR_F_GPIO_Port, LR_F_Pin, 1);
	HAL_GPIO_WritePin(LR_B_GPIO_Port, LR_B_Pin, 0);
	HAL_GPIO_WritePin(LF_F_GPIO_Port, LF_F_Pin, 1);
	HAL_GPIO_WritePin(LF_B_GPIO_Port, LF_B_Pin, 0);
}

void smartcar_back() {
	HAL_GPIO_WritePin(RR_F_GPIO_Port, RR_F_Pin, 0);
	HAL_GPIO_WritePin(RR_B_GPIO_Port, RR_B_Pin, 1);
	HAL_GPIO_WritePin(RF_F_GPIO_Port, RF_F_Pin, 0);
	HAL_GPIO_WritePin(RF_B_GPIO_Port, RF_B_Pin, 1);
	HAL_GPIO_WritePin(LR_F_GPIO_Port, LR_F_Pin, 0);
	HAL_GPIO_WritePin(LR_B_GPIO_Port, LR_B_Pin, 1);
	HAL_GPIO_WritePin(LF_F_GPIO_Port, LF_F_Pin, 0);
	HAL_GPIO_WritePin(LF_B_GPIO_Port, LF_B_Pin, 1);
}

void smartcar_left() {
	HAL_GPIO_WritePin(RR_F_GPIO_Port, RR_F_Pin, 1);
	HAL_GPIO_WritePin(RR_B_GPIO_Port, RR_B_Pin, 0);
	HAL_GPIO_WritePin(RF_F_GPIO_Port, RF_F_Pin, 1);
	HAL_GPIO_WritePin(RF_B_GPIO_Port, RF_B_Pin, 0);
	HAL_GPIO_WritePin(LR_F_GPIO_Port, LR_F_Pin, 0);
	HAL_GPIO_WritePin(LR_B_GPIO_Port, LR_B_Pin, 1);
	HAL_GPIO_WritePin(LF_F_GPIO_Port, LF_F_Pin, 0);
	HAL_GPIO_WritePin(LF_B_GPIO_Port, LF_B_Pin, 1);
}

void smartcar_right() {
	HAL_GPIO_WritePin(RR_F_GPIO_Port, RR_F_Pin, 0);
	HAL_GPIO_WritePin(RR_B_GPIO_Port, RR_B_Pin, 1);
	HAL_GPIO_WritePin(RF_F_GPIO_Port, RF_F_Pin, 0);
	HAL_GPIO_WritePin(RF_B_GPIO_Port, RF_B_Pin, 1);
	HAL_GPIO_WritePin(LR_F_GPIO_Port, LR_F_Pin, 1);
	HAL_GPIO_WritePin(LR_B_GPIO_Port, LR_B_Pin, 0);
	HAL_GPIO_WritePin(LF_F_GPIO_Port, LF_F_Pin, 1);
	HAL_GPIO_WritePin(LF_B_GPIO_Port, LF_B_Pin, 0);
}

void smartcar_stop() {
	HAL_GPIO_WritePin(RR_F_GPIO_Port, RR_F_Pin, 0);
	HAL_GPIO_WritePin(RR_B_GPIO_Port, RR_B_Pin, 0);
	HAL_GPIO_WritePin(RF_F_GPIO_Port, RF_F_Pin, 0);
	HAL_GPIO_WritePin(RF_B_GPIO_Port, RF_B_Pin, 0);
	HAL_GPIO_WritePin(LR_F_GPIO_Port, LR_F_Pin, 0);
	HAL_GPIO_WritePin(LR_B_GPIO_Port, LR_B_Pin, 0);
	HAL_GPIO_WritePin(LF_F_GPIO_Port, LF_F_Pin, 0);
	HAL_GPIO_WritePin(LF_B_GPIO_Port, LF_B_Pin, 0);
}

// --- 초음파 센서 관련 함수들 ---
void delay_us(uint16_t us, TIM_HandleTypeDef *htim) {
	__HAL_TIM_SET_COUNTER(htim, 0);
	while ((__HAL_TIM_GET_COUNTER(htim)) < us);
}

void trig(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin, TIM_HandleTypeDef *htim) {
	HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, 1);
	delay_us(10, htim);
	HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, 0);
}

long unsigned int echo(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin,
		TIM_HandleTypeDef *htim) {
	long unsigned int echo_duration = 0;
	uint32_t start_time;
	uint32_t timeout = 30000;

	start_time = __HAL_TIM_GET_COUNTER(htim);
	while (HAL_GPIO_ReadPin(GPIO_Port, GPIO_Pin) == 0) {
		if (__HAL_TIM_GET_COUNTER(htim) - start_time > timeout) {
			return 0;
		}
	}
	__HAL_TIM_SET_COUNTER(htim, 0);
	while (HAL_GPIO_ReadPin(GPIO_Port, GPIO_Pin) == 1) {
		if (__HAL_TIM_GET_COUNTER(htim) > timeout) {
			return 0;
		}
	}
	echo_duration = __HAL_TIM_GET_COUNTER(htim);

	if (echo_duration >= 240 && echo_duration <= 23000)
		return echo_duration;
	else
		return 0;
}

/**
 * @brief 특정 주파수와 지속시간으로 톤 재생
 * @param frequency: 재생할 주파수 (Hz), 0이면 무음
 * @param duration: 재생 시간 (밀리초)
 */

void play_buzzer(uint16_t frequency, uint16_t duration){
	if(frequency == 0){
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		return;
	}
	/** 주파수 계산:
	 * - TIM3는 APB1 버스에 연결되어 있습니다. (SystemClock_Config()에서 HCLK/2 = 32MHz)
	 * - MX_TIM3_Init()에서 Prescaler는 64-1 (즉, 63)로 설정되어 있습니다.
	 * - 타이머 카운터 클록 = 32,000,000 Hz / (63 + 1) = 500,000 Hz (0.5MHz)
	 * - ARR(주기) = (타이머 카운터 클록 / 원하는 주파수) - 1
	 */
	uint32_t arr_value = 1000000 / frequency - 1;

	__HAL_TIM_SET_AUTORELOAD(&htim3, arr_value);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1 , arr_value / 2);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	osDelay(duration);

	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		if (BluetoothTaskHandle != NULL) {
			g_rx3_command = rx3_data;

			vTaskNotifyGiveFromISR(BluetoothTaskHandle, NULL);
		}
		HAL_UART_Receive_IT(&huart3, &rx3_data, 1);
	}
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	HAL_UART_Receive_IT(&huart3, &rx3_data, 1); // 블루투스
	HAL_UART_Receive_IT(&huart2, &rx2_data, 1);

	//초음파
	HAL_TIM_Base_Start(&htim1);

//	/* Calibrate ADC */
//	HAL_ADCEx_Calibration_Start(&hadc1);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of motorQueue */
  osMessageQDef(motorQueue, 5, uint16_t);
  motorQueueHandle = osMessageCreate(osMessageQ(motorQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of UltrasonicTask */
  osThreadDef(UltrasonicTask, StartUltrasonicTask, osPriorityLow, 0, 128);
  UltrasonicTaskHandle = osThreadCreate(osThread(UltrasonicTask), NULL);

  /* definition and creation of BluetoothTask */
  osThreadDef(BluetoothTask, StartBluetoothTask, osPriorityAboveNormal, 0, 128);
  BluetoothTaskHandle = osThreadCreate(osThread(BluetoothTask), NULL);

  /* definition and creation of MotorTask */
  osThreadDef(MotorTask, StartMotorTask, osPriorityHigh, 0, 128);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

  /* definition and creation of MultiTask */
  osThreadDef(MultiTask, StartMultiTask, osPriorityBelowNormal, 0, 128);
  MultiTaskHandle = osThreadCreate(osThread(MultiTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
  {

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart3.Init.BaudRate = 9600;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Trigger2_Pin|LR_F_Pin|LF_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LR_B_Pin|LF_F_Pin|RF_F_Pin|RF_B_Pin
                          |RR_F_Pin|RR_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo2_Pin */
  GPIO_InitStruct.Pin = Echo2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Trigger2_Pin LR_F_Pin LF_B_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Trigger2_Pin|LR_F_Pin|LF_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo_Pin */
  GPIO_InitStruct.Pin = Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LR_B_Pin LF_F_Pin RF_F_Pin RF_B_Pin
                           RR_F_Pin RR_B_Pin */
  GPIO_InitStruct.Pin = LR_B_Pin|LF_F_Pin|RF_F_Pin|RF_B_Pin
                          |RR_F_Pin|RR_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Trigger_Pin */
  GPIO_InitStruct.Pin = Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trigger_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUltrasonicTask */
/**
* @brief Function implementing the UltrasonicTask thread.
* @param  argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUltrasonicTask */
void StartUltrasonicTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

	uint8_t tx_buffer[6];
	for (;;) {
		// 1. 실제 초음파 센서로 거리를 측정합니다.
		trig(Trigger_GPIO_Port, Trigger_Pin, &htim1);
		long unsigned int echo_time_forward = echo(Echo_GPIO_Port, Echo_Pin, &htim1);

		trig(Trigger2_GPIO_Port, Trigger2_Pin, &htim1);
		long unsigned int echo_time_back = echo(Echo2_GPIO_Port, Echo2_Pin,	&htim1);

		int dist_forward = MAX_DISTANCE;
		int dist_back = MAX_DISTANCE;

		if (echo_time_forward > 0 && echo_time_forward < 23200 ) {
			dist_forward = echo_time_forward / 58; // us를 cm로 변환
		}
		if (echo_time_back > 0 && echo_time_back < 23200 ) {
			dist_back = echo_time_back / 58; // us를 cm로 변환
		}

		g_final_dist_forward = dist_forward;
		g_final_dist_back = dist_back;

		//printf("F: %d cm, B: %d cm\r\n", g_final_dist_forward, g_final_dist_back);

		tx_buffer[0] = 0x02; // SOP
		tx_buffer[1] = 'D';  // CMD
		tx_buffer[2] = (uint8_t) dist_forward; // DATA1 (255cm가 넘지 않으므로 uint8_t로 충분)
		tx_buffer[3] = (uint8_t) dist_back;  // DATA2
		// CHECKSUM: 간단하게 CMD, DATA1, DATA2를 더한 값의 하위 8비트 사용
		tx_buffer[4] = tx_buffer[1] + tx_buffer[2] + tx_buffer[3];
		tx_buffer[5] = 0x03; // EOP
		// huart3 (블루투스)를 통해 6바이트 패킷을 전송
		HAL_UART_Transmit(&huart3, tx_buffer, 6, 10);

		// 2. 거리를 기준으로 장애물 여부를 판단합니다.
		obstacle_detected = (g_final_dist_forward <= MIN_DISTANCE || g_final_dist_back <= MIN_DISTANCE);

		osDelay(100);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBluetoothTask */
/**
* @brief Function implementing the BluetoothTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBluetoothTask */
void StartBluetoothTask(void const * argument)
{
  /* USER CODE BEGIN StartBluetoothTask */
	uint32_t command_to_send;
	/* Infinite loop */
	for (;;) {
		// 블루투스 데이터가 수신될 때까지 무한정 대기 (ISR이 알려줄 때까지 잠듦)
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // pd라는 접두사는 FreeRTOS 내부에서 사용하는 네이밍 규칙

		command_to_send = g_rx3_command;

		// 큐에 명령을 넣습니다.
		if (osMessagePut(motorQueueHandle, command_to_send, 10) == osOK) {
			//printf("BT cmd: %c -> Queue\n", (char)command_to_send);
		}
//		else {
//			//printf("Motor queue full!\n");
//		}
	}
  /* USER CODE END StartBluetoothTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorTask */
	osEvent event;
	char command_received;
	/* Infinite loop */
	for (;;) {
		// 1. motorQueue에 데이터가 들어올 때까지 무한정 대기하며 잠듭니다.
		event = osMessageGet(motorQueueHandle, osWaitForever);

		// 2. 큐에서 메시지를 성공적으로 받으면, RTOS가 Task를 깨우고 아래 코드가 실행됩니다.
		if (event.status == osEventMessage) {
			command_received = (char)event.value.v;
			//printf("Motor Exec: %c\n", command_received);

			// 3. 받은 명령에 따라 모터를 제어합니다.
			switch(command_received){
			  case 'w': smartcar_forward(); break;
			  case 's': smartcar_back(); break;
			  case 'a': smartcar_left(); break;
			  case 'd': smartcar_right();break;
			  case 'f': smartcar_stop(); break;
			}
		}
	}
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartMultiTask */
/**
* @brief Function implementing the MultiTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMultiTask */
void StartMultiTask(void const * argument)
{
  /* USER CODE BEGIN StartMultiTask */
//	/* Infinite loop */
	for (;;) {
		if (obstacle_detected == 1) { //같이 돌리면아노딤  , 조도만하다가 멈춤 , 초음파에 쓰레기기값 들어와버림  왜 ? 40넘는것도들어옴 왜 ? min distance가 안먹히는듯

			if (g_final_dist_forward <= MIN_DISTANCE) {
				play_buzzer(262, 200);
				//osMessagePut(motorQueueHandle, (uint32_t) 'f', 0);
				//printf("f %d \n", g_final_dist_forward);
			} else if (g_final_dist_back <= MIN_DISTANCE) {
				play_buzzer(440, 200);
				//osMessagePut(motorQueueHandle, (uint32_t) 'f', 0);
				//printf("d %d \n", g_final_dist_back);
			}
			obstacle_detected = 0;
		}


		osDelay(50);
	}
  /* USER CODE END StartMultiTask */
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
