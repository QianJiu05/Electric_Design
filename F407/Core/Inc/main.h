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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
struct node{
  float votage;
  float current;
};

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

/* USER CODE BEGIN Private defines */
  /* PID para*/
#define AUTO_RELOAD_VALUE 1679
#define MAX_OUTPUT 100.f
#define MIN_OUTPUT 0.0f
  /* ADC para*/
#define ADC_CHANNEL_NUM       16
#define VREF                  15

#define INPUT_CURRENT         1

#define DC_OUT_VOLTAGE_1      2//DCAC OUTPUT
#define DC_OUT_CURRENT_1      3
#define DC_OUT_CURRENT_2      5
#define DC_OUT_VOLTAGE_2      4
#define DC_OUT_CURRENT_3      0
#define DC_OUT_VOLTAGE_3      6

#define AC_IN_VOLTAGE_1       (8 - 1)//ACDC INPUT
#define AC_IN_CURRENT_1       (9 - 1)
#define AC_IN_VOLTAGE_2       (10 - 1)
#define AC_IN_CURRENT_2       (11 - 1)
#define AC_IN_VOLTAGE_3       (12 - 1)
#define AC_IN_CURRENT_3       (13 - 1)

#define CURRENT_REF_CURRENT   (14 - 1)
#define CURRENT_REF_VOLTAGE   (15 - 1)


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
