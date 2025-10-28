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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "lcd.h"

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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// MPU-6050 I2C 주소
#define MPU6050_ADDR 0xD0 // 0x68 << 1

// MPU-6050 내부 레지스터 주소
#define PWR_MGMT_1_REG 0x6B
#define ACCEL_XOUT_H_REG 0x3B

// I2C 핸들러
extern I2C_HandleTypeDef hi2c1;

// 센서 raw 데이터 저장 변수
int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

// 계산된 각도 저장 변수
float Ax, Ay, Az, Gx, Gy, Gz;
float roll, pitch;
float result_roll, result_pitch;

// 시간 측정 변수
uint32_t last_time;
float dt = 0.0;

uint8_t Rec_Data[14]; // 레지스터에서 데이터 저장 (IMU)

volatile uint8_t i2c_flag;

//메세지 받기
#define PACKET_SIZE 6
// 파서의 상태를 나타내는 열거형
typedef enum {
	STATE_WAIT_SOP, STATE_RECEIVE_DATA
} ParserState_t;

// 파서의 현재 상태를 저장하는 변수
ParserState_t g_parser_state = STATE_WAIT_SOP;
// 수신된 패킷 데이터를 임시로 저장할 버퍼
uint8_t g_rx_buffer[PACKET_SIZE];
// 버퍼에 몇 개의 바이트가 쌓였는지 나타내는 인덱스
uint8_t g_rx_buffer_index = 0;
// UART 수신을 위한 1바이트 버퍼
uint8_t g_master_rx_byte = 0;

int forward_dist;
int back_dist;

int last_data_f = 0;
int last_data_b = 0;

//volatile uint8_t lcd_draw_flag = 0;

int forward_dist_display = 0;
int back_dist_display = 0;

uint32_t last_lcd_update_time = 0;

volatile uint8_t stop_button_pressed = 0; // 버튼 정지 상태 플래그 추가

uint8_t tran_data; //master 명령 -> slave

 typedef enum{
	 NORMAL_Display,
	 WARNING_Display
 }DisplayState;

 DisplayState current_display_state = NORMAL_Display;

char lcd_buf[20]; // LCD에 데이터넣을곳
char display_buf[17]; // lcd 공백 맞추기


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
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

void IMU_Calculate() {

	// 시간 간격(dt) 계산 - 보정에 필요함
	uint32_t current_time = HAL_GetTick();
	dt = (current_time - last_time) / 1000.0f;
	last_time = current_time;

	// --- 1. 센서 데이터 읽기 ---
	// 가속도(3축), 온도, 자이로(3축) 데이터를 한 번에 14바이트 읽어옴
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14,
			100);

	/**  16비트 데이터로 변환
	 * i2c는 8bit 밖에 전송을 못함 그래서 상위 8bit 하위 8bit 쪼개서 보냄  이데이터를 합치는 과정
	 * rec_data[0] 상위 8bit [15:8] / rec_data[1] 하위[7:0] bit
	 * 상위비트 쉬프트 연산으로 8칸 이동시킴 ex) 1011 -> << 8(스프트 8) 101101000
	 * 하위 8bit ex) 00001100   둘이 or 연산
	 * 10110100 00000000 | 0000000 00001100  => 10110100 00001100 원래 16bit 복원
	 */
	Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
	Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
	Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
	Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);

	// --- 2. 가속도 센서로 Roll, Pitch 각도 계산 ---
	// raw 데이터를 실제 물리량으로 변환 (옵션, 여기서는 단순 계산)
	Ax = Accel_X_RAW / 16384.0f;
	Ay = Accel_Y_RAW / 16384.0f;
	Az = Accel_Z_RAW / 16384.0f;

	// atan2 함수를 이용해 각도 계산 (라디안 -> 각도 변환)
	roll = atan2(Ay, Az) * 180.0f / M_PI;
	pitch = atan2(-Ax, sqrt(Ay * Ay + Az * Az)) * 180.0f / M_PI;

	// --- 3. 자이로 센서 값으로 각도 변화량 계산 ---
	// raw 데이터를 각속도(degree/s)로 변환
	//Gx = Gyro_X_RAW / 131.0f;
	//Gy = Gyro_Y_RAW / 131.0f;
	//Gz = Gyro_Z_RAW / 131.0f;

	Gx = Gyro_X_RAW / 32.8f;
	Gy = Gyro_Y_RAW / 32.8f;
	Gz = Gyro_Z_RAW / 32.8f;
	// --- 4. 상보 필터 적용 ---
	// 자이로 값으로 각도를 적분하고, 가속도 값으로 보정
	//result_roll = 0.98 * (result_roll + Gx * dt) + 0.02 * roll;
	//result_pitch = 0.98 * (result_pitch + Gy * dt) + 0.02 * pitch;

	result_roll = 0.9 * (result_roll + Gx * dt) + 0.1 * roll; // 0.98 자이로스코프(각속도)  0.02 엑셀로미터(가속도) - 상보 필터 기본 디폴트 98 * 2 근데 바꿈 90% * 1%
	result_pitch = 0.9 * (result_pitch + Gy * dt) + 0.1 * pitch;

}

/**
 * @brief 1바이트의 UART 데이터를 받아 패킷을 파싱하는 함수
 * @param rx_data: UART로부터 수신된 1바이트 데이터6
 *
 */
void parse_uart_byte(uint8_t rx_data) {

	switch (g_parser_state) {
	case STATE_WAIT_SOP:
		// 패킷의 시작(SOP)을 기다리다가, SOP(0x02)를 만나면
		if (rx_data == 0x02) {
			// 버퍼를 초기화하고, SOP를 버퍼에 저장
			g_rx_buffer_index = 0;
			g_rx_buffer[g_rx_buffer_index++] = rx_data;
			// 상태를 '데이터 수신 중'으로 변경
			g_parser_state = STATE_RECEIVE_DATA;
		}
		break;

	case STATE_RECEIVE_DATA:
		// 나머지 데이터를 버퍼에 순서대로 저장
		g_rx_buffer[g_rx_buffer_index++] = rx_data;

		// 패킷 사이즈(6바이트)만큼 데이터가 모두 수신되었다면
		if (g_rx_buffer_index >= PACKET_SIZE) {
			// --- 1. 패킷 유효성 검증 ---
			uint8_t sop = g_rx_buffer[0];
			uint8_t cmd = g_rx_buffer[1];
			uint8_t checksum = g_rx_buffer[4];
			uint8_t eop = g_rx_buffer[5];

			// Checksum 계산 (CMD + DATA1 + DATA2)
			uint8_t calculated_checksum = g_rx_buffer[1] + g_rx_buffer[2]
					+ g_rx_buffer[3];

			// SOP, EOP, Checksum이 모두 일치하는 유효한 패킷이라면
			if (sop == 0x02 && eop == 0x03 && checksum == calculated_checksum) {
				// --- 2. 데이터 추출 및 사용 ---
				if (cmd == 'D') { // CMD가 '거리' 데이터가 맞다면
					forward_dist = g_rx_buffer[2];
					back_dist = g_rx_buffer[3];

					// Store data for display and set flag
					forward_dist_display = forward_dist;
					back_dist_display = back_dist;
					//lcd_draw_flag = 1; // Signal new data is ready

					last_data_f = forward_dist;
					last_data_b = back_dist;

					//printf(" %d |  %d \n", 	forward_dist, back_dist);
				}
			}

			// --- 3. 다음 패킷을 받기 위해 상태 초기화 ---
			g_rx_buffer_index = 0;
			g_parser_state = STATE_WAIT_SOP;
		}
		break;
	}
}

void rc_control(void){
	if ((int) result_pitch > 80) {
		tran_data = 'w';
		//LCD_DrawString(10, 75, "Command : FORWARD ", YELLOW, BLACK);
		//sprintf(cmd_buf, "Command: FORWARD ");

	}
	else if ((int) result_pitch < -80) {
		tran_data = 's';
		//LCD_DrawString(10, 75, "Command : BACK   ", YELLOW, BLACK);
		//sprintf(cmd_buf, "Command: LEFT    ");
	}

	else if ((int) result_roll < -70) {
		tran_data = 'a'; // f103 - w
		//LCD_DrawString(10, 75, "Command : LEFT   ", YELLOW, BLACK);
		//sprintf(cmd_buf, "Command: RIGHT   ");

	}
	else if ((int) result_roll > 70) {
		tran_data = 'd';
		//LCD_DrawString(10, 75, "Command : RIGHT   ", YELLOW, BLACK);
		//sprintf(cmd_buf, "Command: RIGHT   ");

	}

	else if ((abs((int) result_roll) < 30) && (abs((int) result_pitch) < 50)) {
		tran_data = 'f';
		//LCD_DrawString(10, 75, "Command : Stop   ", YELLOW, BLACK);
		//sprintf(cmd_buf, "Command: STOP    ");
	}

	//printf("%c : pitch = %d | roll = %d \n",(char)tran_data ,(int) result_pitch,(int) result_roll);

	HAL_UART_Transmit(&huart1, &tran_data, 1, 100);
	//printf("Transmit char: %c, dec: %d\n", tran_data, tran_data);
	//LCD_DrawString(10, 75, cmd_buf, YELLOW, BLACK);


}

void LCD_Display_update(void) {
	DisplayState next_display_state =
			(forward_dist_display < 30 || back_dist_display < 30) ?
					WARNING_Display : NORMAL_Display;

	if (next_display_state != current_display_state) {

		if (next_display_state == WARNING_Display) {
			LCD_Fill(RED);
			LCD_DrawString(20, 60, "!! WARNING !!", YELLOW, RED);
		} else {
			LCD_Fill(BLACK);
			LCD_DrawString(10, 30, "Front Distance", WHITE, BLACK);
			LCD_DrawString(10, 70, "Rear Distance", WHITE, BLACK);

		}
		current_display_state = next_display_state;
	}

	//		  %-16.16s의 의미는 다음과 같습니다.
	//		  - .16 (정밀도): 원본(lcd_buf)에서 최대 16글자까지만 가져온다. (오버플로우 방지)
	//		  - -16 (너비): 위에서 가져온 글자를 16칸에 맞춰 출력하되, 모자라면 나머지를 공백으로 채운다. (잔상 제거)
	if (current_display_state == NORMAL_Display) {
		sprintf(lcd_buf, "=>  %d  cm", forward_dist_display);
		snprintf(display_buf, sizeof(display_buf), "%-16.16s", lcd_buf);
		LCD_DrawString(10, 45, display_buf, YELLOW, BLACK);

		sprintf(lcd_buf, "=>  %d  cm", back_dist_display);
		snprintf(display_buf, sizeof(display_buf), "%-16.16s", lcd_buf);
		LCD_DrawString(10, 85, display_buf, YELLOW, BLACK);

	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {

		uint8_t wake_up_data;

		// MPU-6050이 연결되었는지 확인
		HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 1, 100);

		// MPU-6050을 Sleep 모드에서 깨우기
		wake_up_data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &wake_up_data, 1, 100);

		HAL_I2C_Mem_Read_IT(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14);
	}

}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == I2C1) {
		i2c_flag = 1;
		//printf("i2c ok\n");
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// Master가 사용하는 UART 포트 (예: USART2)인지 확인
	if (huart->Instance == USART1) {
		// 수신된 1바이트를 파서 함수로 넘겨줌
		parse_uart_byte(g_master_rx_byte);
		// 다음 1바이트 수신을 위해 인터럽트 다시 활성화
		HAL_UART_Receive_IT(huart, &g_master_rx_byte, 1);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == BUtton_STOP_Pin) {
		static uint32_t temp_time = 0;
		if (HAL_GetTick() - temp_time > 200) {
			stop_button_pressed = !stop_button_pressed; // 플래그 상태 반전 (토글)
			tran_data = 'f';

			temp_time = HAL_GetTick();

		}
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */


	uint8_t wake_up_data;

	// MPU-6050이 연결되었는지 확인
	HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 1, 100);

	// MPU-6050을 Sleep 모드에서 깨우기
	wake_up_data = 0;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &wake_up_data, 1,
			100);

	uint8_t gyro_config = 0x10; // FS_SEL = 2 // 민감도 바꾸는거   Register 27(0x1B) – Gyroscope Configuration
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &gyro_config, 1, 100);

	last_time = HAL_GetTick(); // 초기 시간 설정

	HAL_TIM_Base_Start_IT(&htim2);

	HAL_UART_Receive_IT(&huart1, &g_master_rx_byte, 1);

	LCD_Init();      // LCD 하드웨어 초기화
	LCD_Fill(BLACK); // LCD 화면을 검은색으로 채우기  - lcd 색상 default가 반전임

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		/** 플래그 이용해서 버튼눌리면 IMU 무시하고 STOP
		 * 논블로킹 슈퍼루프 방식으로 구현
		 */
		if (stop_button_pressed == 1) {
			// 정지 버튼이 눌렸으면 IMU 제어를 무시하고 'f' 명령을 계속 전송
			tran_data = 'f';
			HAL_UART_Transmit(&huart1, &tran_data, 1, 100);
		} else {
			// 정지 버튼이 눌리지 않았으면 정상적인 IMU 기반 제어
			if (i2c_flag == 1) {
				i2c_flag = 0;

				IMU_Calculate(); // imu 계산
				rc_control(); // slave한데 블루투스로 rc카 제어 신호보내주는 함수

			}
		}

		if (HAL_GetTick() - last_lcd_update_time > 1000) { // 100ms = 0.1s

			LCD_Display_update(); // lcd_updatae 함수

			last_lcd_update_time = HAL_GetTick();
		}
		HAL_Delay(10);



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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RES_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RES_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_RES_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUtton_STOP_Pin */
  GPIO_InitStruct.Pin = BUtton_STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUtton_STOP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
