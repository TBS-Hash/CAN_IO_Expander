/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define GPO_Direction_M1_Pin GPIO_PIN_13
#define GPO_Direction_M1_GPIO_Port GPIOC
#define GPO_Direction_M2_Pin GPIO_PIN_14
#define GPO_Direction_M2_GPIO_Port GPIOC
#define GPI_Sensor_3_Pin GPIO_PIN_15
#define GPI_Sensor_3_GPIO_Port GPIOC
#define GPI_Sensor_2_Pin GPIO_PIN_0
#define GPI_Sensor_2_GPIO_Port GPIOA
#define GPI_Sensor_1_Pin GPIO_PIN_1
#define GPI_Sensor_1_GPIO_Port GPIOA
#define SM_GPIO25_Pin GPIO_PIN_12
#define SM_GPIO25_GPIO_Port GPIOB
#define SM_GPIO25B13_Pin GPIO_PIN_13
#define SM_GPIO25B13_GPIO_Port GPIOB
#define SM_GPIO26_Pin GPIO_PIN_14
#define SM_GPIO26_GPIO_Port GPIOB
#define SM_GPIO27_Pin GPIO_PIN_3
#define SM_GPIO27_GPIO_Port GPIOB
#define SM_GPIO28_Pin GPIO_PIN_4
#define SM_GPIO28_GPIO_Port GPIOB
#define SM_GPIO29_Pin GPIO_PIN_5
#define SM_GPIO29_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
