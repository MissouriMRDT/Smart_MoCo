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

#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_utils.h"


#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
volatile extern uint64_t sysTickOffset;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint64_t GetTick(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO_Pin LL_GPIO_PIN_8
#define GPIO_GPIO_Port GPIOB
#define IN_B_Pin LL_GPIO_PIN_0
#define IN_B_GPIO_Port GPIOF
#define IN_A_Pin LL_GPIO_PIN_1
#define IN_A_GPIO_Port GPIOF
#define CS_Pin LL_GPIO_PIN_0
#define CS_GPIO_Port GPIOA
#define ENC_B_Pin LL_GPIO_PIN_1
#define ENC_B_GPIO_Port GPIOA
#define LIM_A_Pin LL_GPIO_PIN_2
#define LIM_A_GPIO_Port GPIOA
#define LIM_B_Pin LL_GPIO_PIN_3
#define LIM_B_GPIO_Port GPIOA
#define PWM_Pin LL_GPIO_PIN_4
#define PWM_GPIO_Port GPIOA
#define ENC_A_Pin LL_GPIO_PIN_5
#define ENC_A_GPIO_Port GPIOA
#define LED_R_Pin LL_GPIO_PIN_6
#define LED_R_GPIO_Port GPIOA
#define LED_G_Pin LL_GPIO_PIN_7
#define LED_G_GPIO_Port GPIOA
#define LED_B_Pin LL_GPIO_PIN_1
#define LED_B_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0                                                   \
  ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,                 \
                              4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1                                                   \
  ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,                 \
                              3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2                                                   \
  ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,                 \
                              2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3                                                   \
  ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,                 \
                              1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4                                                   \
  ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,                 \
                              0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */
#define TICKS_PER_S 8000000
#define TICKS_PER_MS 8000
#define TICKS_PER_US 8

#define TIM_SCHEDULER TIM1
#define TIM_ENCODER TIM2
#define TIM_LED TIM3
#define TIM_MOTOR TIM14
#define TIM_GPIO TIM16
#define TIM_OUTPUT TIM17
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
