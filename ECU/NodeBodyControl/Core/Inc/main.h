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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAN_CS_Pin GPIO_PIN_0
#define CAN_CS_GPIO_Port GPIOB
#define XNT_Pin GPIO_PIN_10
#define XNT_GPIO_Port GPIOB
#define XNP_Pin GPIO_PIN_11
#define XNP_GPIO_Port GPIOB
#define GAP_Pin GPIO_PIN_12
#define GAP_GPIO_Port GPIOB
#define SET_Pin GPIO_PIN_13
#define SET_GPIO_Port GPIOB
#define LIMIT_Pin GPIO_PIN_14
#define LIMIT_GPIO_Port GPIOB
#define RES_Pin GPIO_PIN_15
#define RES_GPIO_Port GPIOB
#define Den_pha_Pin GPIO_PIN_3
#define Den_pha_GPIO_Port GPIOB
#define Hazard_Pin GPIO_PIN_4
#define Hazard_GPIO_Port GPIOB
#define Den_cos_Pin GPIO_PIN_5
#define Den_cos_GPIO_Port GPIOB
#define Echo_Pin GPIO_PIN_6
#define Echo_GPIO_Port GPIOB
#define Trig_Pin GPIO_PIN_7
#define Trig_GPIO_Port GPIOB
#define Cruise_control_Pin GPIO_PIN_8
#define Cruise_control_GPIO_Port GPIOB
#define Led_can_Pin GPIO_PIN_9
#define Led_can_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
