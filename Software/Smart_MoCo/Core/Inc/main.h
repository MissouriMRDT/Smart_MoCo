/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define IN_A_Pin GPIO_PIN_0
#define IN_A_GPIO_Port GPIOA
#define IN_B_Pin GPIO_PIN_1
#define IN_B_GPIO_Port GPIOA
#define SEL0_Pin GPIO_PIN_2
#define SEL0_GPIO_Port GPIOA
#define SEL1_Pin GPIO_PIN_3
#define SEL1_GPIO_Port GPIOA
#define MS_EN_Pin GPIO_PIN_4
#define MS_EN_GPIO_Port GPIOA
#define MS_IN_Pin GPIO_PIN_5
#define MS_IN_GPIO_Port GPIOA
#define PWM_Pin GPIO_PIN_7
#define PWM_GPIO_Port GPIOA
#define ENC_ABS_Pin GPIO_PIN_14
#define ENC_ABS_GPIO_Port GPIOB
#define ENC_I_Pin GPIO_PIN_15
#define ENC_I_GPIO_Port GPIOB
#define ENC_B_Pin GPIO_PIN_8
#define ENC_B_GPIO_Port GPIOA
#define ENC_A_Pin GPIO_PIN_9
#define ENC_A_GPIO_Port GPIOA
#define LIM_A_Pin GPIO_PIN_3
#define LIM_A_GPIO_Port GPIOB
#define LIM_B_Pin GPIO_PIN_4
#define LIM_B_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_5
#define LED_R_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_8
#define LED_G_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_9
#define LED_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
