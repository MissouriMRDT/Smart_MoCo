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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum ControlMode ControlMode;
enum ControlMode {
  CONTROL_MODE_STOP,
  CONTROL_MODE_OPEN_LOOP,
  CONTROL_MODE_POSITION,
  CONTROL_MODE_VELOCITY,
  CONTROL_MODE_CURRENT,
  CONTROL_MODE_CALIBRATING
};
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
#define LED_STATUS_Pin GPIO_PIN_0
#define LED_STATUS_GPIO_Port GPIOF
#define DIAG_A_Pin GPIO_PIN_1
#define DIAG_A_GPIO_Port GPIOF
#define CS_Pin GPIO_PIN_0
#define CS_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_1
#define ENC_B_GPIO_Port GPIOA
#define LIM_A_Pin GPIO_PIN_2
#define LIM_A_GPIO_Port GPIOA
#define LIM_B_Pin GPIO_PIN_3
#define LIM_B_GPIO_Port GPIOA
#define PWM_Pin GPIO_PIN_4
#define PWM_GPIO_Port GPIOA
#define ENC_A_Pin GPIO_PIN_5
#define ENC_A_GPIO_Port GPIOA
#define IN_B_Pin GPIO_PIN_7
#define IN_B_GPIO_Port GPIOA
#define IN_A_Pin GPIO_PIN_1
#define IN_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define MESSAGE_ID_POSITION 0x30
#define MESSAGE_ID_POSITION_CALIBRATED 0x31
#define MESSAGE_ID_ERROR 0x32
#define MESSAGE_ID_ECHO_REPLY 0x3F
#define MESSAGE_ID_STOP 0x00
#define MESSAGE_ID_RAMP_RATE 0x01
#define MESSAGE_ID_PI 0x02
#define MESSAGE_ID_D 0x03
#define MESSAGE_ID_IGNORE_LIMIT 0x04
#define MESSAGE_ID_SOFT_LIMIT 0x05
#define MESSAGE_ID_CALIBRATE 0x06
#define MESSAGE_ID_DEBUG 0x07
#define MESSAGE_ID_DUTY_CYCLE_RANGE 0x08
#define MESSAGE_ID_ECHO_REQUEST 0x0F
#define MESSAGE_ID_OPEN_LOOP 0x10
#define MESSAGE_ID_TARGET_POSITION 0x11
#define MESSAGE_ID_TARGET_VELOCITY 0x12
#define MESSAGE_ID_TARGET_CURRENT 0x13
#define MESSAGE_ID_DEBUG_OFFSET 0x7F0

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
