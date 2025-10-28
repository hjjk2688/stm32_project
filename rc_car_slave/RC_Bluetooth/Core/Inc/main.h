/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Echo2_Pin GPIO_PIN_4
#define Echo2_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Echo_Pin GPIO_PIN_0
#define Echo_GPIO_Port GPIOB
#define LR_B_Pin GPIO_PIN_10
#define LR_B_GPIO_Port GPIOB
#define Trigger_Pin GPIO_PIN_7
#define Trigger_GPIO_Port GPIOC
#define Trigger2_Pin GPIO_PIN_8
#define Trigger2_GPIO_Port GPIOA
#define LR_F_Pin GPIO_PIN_9
#define LR_F_GPIO_Port GPIOA
#define LF_B_Pin GPIO_PIN_10
#define LF_B_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LF_F_Pin GPIO_PIN_3
#define LF_F_GPIO_Port GPIOB
#define RF_F_Pin GPIO_PIN_4
#define RF_F_GPIO_Port GPIOB
#define RF_B_Pin GPIO_PIN_5
#define RF_B_GPIO_Port GPIOB
#define RR_F_Pin GPIO_PIN_8
#define RR_F_GPIO_Port GPIOB
#define RR_B_Pin GPIO_PIN_9
#define RR_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
