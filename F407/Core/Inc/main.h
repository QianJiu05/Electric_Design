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

typedef struct TRIPLE_POWER {
  float CURRENT_A;
  float CURRENT_B;
  float CURRENT_C;

  float VOLTAGE_A;
  float VOLTAGE_B;
  float VOLTAGE_C;
}triple_power;
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
#define VREF                  3.3f

#define ADC_CHANNEL_NUM       16

#define DC_INPUT_CURRENT         0//DCAC INPUT

#define DC_OUT_VOLTAGE_A      (15 - 1)//DCAC OUTPUT
#define DC_OUT_CURRENT_A      (9 - 1)
#define DC_OUT_CURRENT_B      (14 - 1)
#define DC_OUT_VOLTAGE_B      (8 - 1)
#define DC_OUT_CURRENT_C      4
#define DC_OUT_VOLTAGE_C      6

#define AC_IN_VOLTAGE_A       (13 - 1)//ACDC INPUT
#define AC_IN_CURRENT_A       (12 - 1)
#define AC_IN_VOLTAGE_B       (11 - 1)
#define AC_IN_CURRENT_B       (10 - 1)
#define AC_IN_VOLTAGE_C       5
#define AC_IN_CURRENT_C       3

#define AC_OUT_VOLTAGE   1//DCAC OUTPUT
#define AC_OUT_CURRENT   2



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
