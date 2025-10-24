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
#include <stdbool.h>
#include <stdint.h>
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
ADC_HandleTypeDef hadc;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
// Comment out to use absolute PWM encoder on PA5 and uncomment to use
// quadrature encoder on PA1 and PA5.
// #define QUADRATURE_ENCODER 1

// Absolute encoder resolution (steps traveled as PWM duty cycle increases from
// 0% to 100%)
const uint32_t ABSOLUTE_ENCODER_RESOLUTION = 1024;

// (0x7F to 0x00) << 4 The high two nybbles of all CAN IDs associated with this
// device. This should be unique for each Smart MoCo on the CAN bus.
const uint32_t MOCO_ID = 0x0B << 4;

// Telemetry interval (ms)
const uint32_t TELEMETRY_INTERVAL = 500;

// Missing parameter request interval (ms)
const uint32_t PARAMETER_REQUEST_INTERVAL = 500;

#ifndef QUADRATURE_ENCODER
volatile uint32_t encoderDutyCycle = 0;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM14_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static void reportCommandError(uint8_t commandID);
static double pide(double error, double P, double I, double D, double errorGain,
                   double *lastError, double *integralError, uint32_t deltaT);
static void lowPass(double in, double *out, double alpha, double deltaT);
uint16_t u16FromBytes(uint8_t *bytes);
int16_t i16FromBytes(uint8_t *bytes);
uint32_t u32FromBytes(uint8_t *bytes);
int32_t i32FromBytes(uint8_t *bytes);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM14_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
#ifdef QUADRATURE_ENCODER
  // Quadrature Encoder
  htim2.Instance->CNT = 0x800000000000;
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
#else
  // Absolute Encoder
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
#endif

  // Elapsed time counter
  HAL_TIM_Base_Start(&htim1);

  CAN_FilterTypeDef filter = {.FilterIdHigh = 0x0000,
                              .FilterIdLow = 0x0000,
                              .FilterMaskIdHigh = 0x0000,
                              .FilterMaskIdLow = 0x0000,
                              .FilterFIFOAssignment = CAN_FILTER_FIFO0,
                              .FilterBank = 0,
                              .FilterMode = CAN_FILTERMODE_IDMASK,
                              .FilterScale = CAN_FILTERSCALE_16BIT,
                              .FilterActivation = CAN_FILTER_ENABLE};
  HAL_CAN_ConfigFilter(&hcan, &filter);
  HAL_CAN_Start(&hcan);
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  // Controller state
  ControlMode controlMode = CONTROL_MODE_STOP;
  bool ignoreLimit = false;
  uint32_t nextReportTime = 0; // ms
  bool softLimitA = false;
  bool softLimitB = false;
  bool fault = false;
  uint32_t lastTick = 0;
  double lastError = 0;
  double integralError = 0;
  double pwm = 0;
#ifdef QUADRATURE_ENCODER
  uint32_t zeroPosition = 0x800000000000;
#else
  int32_t encoderRotations = 0;
  int32_t zeroPositionRotations = 0;
  uint32_t zeroPositionDutyCycle = encoderDutyCycle;
  uint32_t encoderLastDutyCycle = 0;
#endif

  // Parameter tracking
  uint16_t setParameters = 0;
  bool lastPIDUsed = false;
  uint16_t missingParameters = 0;
  uint8_t nextMissingParameterRequestID = 0;
  uint32_t nextParameterRequestTime = 0;
  // Set PID logic
  // +-----+      +---+         +----+
  // |Unset|-PID->|Set|-Target->|Used|
  // +-----+      +---+         +----+
  //   ^ ^--Reset--/ ^----PID----/ |
  //   \---Reset OR Other Target---/

  // Controller feedback
  int32_t currentPosition = 0; // step
  double currentVelocity = 0;  // step/sec
  uint16_t currentCurrent = 0; // ADC
  bool limitA = false;
  bool limitB = false;

  // Controller configuration
  double P = 0.0;
  double I = 0.0;
  double D = 0.0;
  double errorGain = 1.0;
  double alpha = 0.0;
  uint32_t limitAPosition = 0;              // step
  uint32_t limitBPosition = UINT32_MAX;     // step
  uint32_t softLimitAPosition = 0;          // step
  uint32_t softLimitBPosition = UINT32_MAX; // step

  // Controller input
  // Open Loop: 1/32768 i16
  // Position: step
  // Velocity: step/sec
  // Current: ADC
  int32_t target = 0;

  // Controller output
  bool motorA = false;
  bool motorB = false;
  uint16_t motorPWM = 0; // 1/65536
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    // Process a single received CAN message
    if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0 &&
        HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData) ==
            HAL_OK) {
      bool commandOK = true;
      if (rxHeader.RTR == CAN_RTR_DATA) {
        switch (rxHeader.StdId & 0x00F) {
        case MESSAGE_ID_TARGET:
          if (rxHeader.DLC < 1) {
            commandOK = false;
            break;
          }
          switch (rxData[0] & 0b11111110) {
          case MESSAGE_SID_OPEN_LOOP:
            if (rxHeader.DLC < 3) {
              commandOK = false;
              break;
            }
            ignoreLimit = rxData[0] & 0b00000001;
            controlMode = CONTROL_MODE_OPEN_LOOP;
            target = i16FromBytes(&rxData[1]);
            break;
          case MESSAGE_SID_POSITION:
            if (rxHeader.DLC < 7) {
              commandOK = false;
              break;
            }
            if (lastPIDUsed && controlMode != CONTROL_MODE_POSITION) {
              setParameters &= ~(1 << MESSAGE_ID_PID);
              lastPIDUsed = false;
            }
            controlMode = CONTROL_MODE_POSITION;
            ignoreLimit = rxData[0] & 0b00000001;
            errorGain = (double)i32FromBytes(&rxData[1]) / 256;
            target = i32FromBytes(&rxData[3]);
            break;
          case MESSAGE_SID_VELOCITY:
            if (rxHeader.DLC < 7) {
              commandOK = false;
              break;
            }
            if (lastPIDUsed && controlMode != CONTROL_MODE_VELOCITY) {
              setParameters &= ~(1 << MESSAGE_ID_PID);
              lastPIDUsed = false;
            }
            controlMode = CONTROL_MODE_VELOCITY;
            ignoreLimit = rxData[0] & 0b00000001;
            errorGain = (double)i32FromBytes(&rxData[1]) / 256;
            target = i32FromBytes(&rxData[3]);
            break;
          case MESSAGE_SID_CURRENT:
            if (rxHeader.DLC < 5) {
              commandOK = false;
              break;
            }
            if (lastPIDUsed && controlMode != CONTROL_MODE_CURRENT) {
              setParameters &= ~(1 << MESSAGE_ID_PID);
              lastPIDUsed = false;
            }
            controlMode = CONTROL_MODE_CURRENT;
            ignoreLimit = rxData[0] & 0b00000001;
            errorGain = (double)i32FromBytes(&rxData[1]) / 256;
            target = i16FromBytes(&rxData[3]);
            break;
          default:
            break;
          }
          break;
        case MESSAGE_ID_SMOOTHING:
          if (rxHeader.DLC < 1) {
            commandOK = false;
            break;
          }
          alpha = (double)u16FromBytes(&rxData[0]) / 65536;
          setParameters |= 1 << MESSAGE_ID_SMOOTHING;
          break;
        case MESSAGE_ID_PID:
          if (rxHeader.DLC < 8) {
            commandOK = false;
            break;
          }
          if (lastPIDUsed)
            lastPIDUsed = false;
          P = ((double)u16FromBytes(&rxData[0])) / 256;
          I = ((double)u16FromBytes(&rxData[2])) / 256;
          D = ((double)u16FromBytes(&rxData[4])) / 256;
          errorGain = ((double)u16FromBytes(&rxData[6])) / 256;
          setParameters |= 1 << MESSAGE_ID_PID;
          break;
        case MESSAGE_ID_LIMIT:
          // Set Limit Switch A Position
          if (rxHeader.DLC < 8) {
            commandOK = false;
            break;
          }
          uint32_t a0 = i32FromBytes(&rxData[0]);
          uint32_t b0 = i32FromBytes(&rxData[4]);
          if (a0 >= b0) {
            // Limit A must be less than limit B
            commandOK = false;
            break;
          }
          limitAPosition = a0;
          limitBPosition = b0;
          setParameters |= 1 << MESSAGE_ID_LIMIT;
          break;
        case MESSAGE_ID_SOFT_LIMIT:
          if (rxHeader.DLC < 8) {
            commandOK = false;
            break;
          }
          uint32_t a1 = i32FromBytes(&rxData[0]);
          uint32_t b1 = i32FromBytes(&rxData[4]);
          if (a1 >= b1) {
            // Limit A must be less than limit B
            commandOK = false;
            break;
          }
          softLimitAPosition = a1;
          softLimitBPosition = b1;
          setParameters |= 1 << MESSAGE_ID_SOFT_LIMIT;
          break;
        case MESSAGE_ID_CALIBRATE:
          if (rxHeader.DLC < 2) {
            commandOK = false;
            break;
          }
          controlMode = CONTROL_MODE_CALIBRATING;
          target = i16FromBytes(&rxData[0]);
          break;
        case MESSAGE_ID_STOP:
          controlMode = CONTROL_MODE_STOP;
          P = 0.0;
          I = 0.0;
          D = 0.0;
          errorGain = 1.0;
          alpha = 0.0;
          target = 0;
          lastPIDUsed = false;
          setParameters = 0;
          break;
        case MESSAGE_ID_ECHO_REQUEST: {
          CAN_TxHeaderTypeDef txHeader = {.StdId =
                                              MOCO_ID | MESSAGE_ID_ECHO_REPLY,
                                          .IDE = CAN_ID_STD,
                                          .RTR = CAN_RTR_DATA,
                                          .DLC = rxHeader.DLC,
                                          .TransmitGlobalTime = DISABLE};
          uint32_t txMailbox;
          HAL_CAN_AddTxMessage(&hcan, &txHeader, rxData, &txMailbox);
        } break;
        default:
          break;
        }
      }
      if (!commandOK) {
        reportCommandError(rxHeader.StdId & 0x00F);
      }
    }

    // Update controller
    uint32_t currentHALTick = HAL_GetTick();    // ms
    uint32_t currentTick = htim1.Instance->CNT; // 0.5 us
    double deltaT =
        (double)(currentTick >= lastTick ? currentTick - lastTick
                                         : 65535 - lastTick + currentTick) /
        2000000;
    lastTick = currentTick;
    int32_t lastPosition = currentPosition;
#ifdef QUADRATURE_ENCODER
    // Quadrature Encoder
    currentPosition = htim2.Instance->CNT - zeroPosition;
#else
    // Absolute Encoder
    uint32_t currentEncoderDutyCycle = encoderDutyCycle;
    if (encoderLastDutyCycle > ABSOLUTE_ENCODER_RESOLUTION * 0.8 &&
        currentEncoderDutyCycle < ABSOLUTE_ENCODER_RESOLUTION * 0.2) {
      // High to low rollover
      encoderRotations++;
    } else if (encoderLastDutyCycle < ABSOLUTE_ENCODER_RESOLUTION * 0.2 &&
               currentEncoderDutyCycle > ABSOLUTE_ENCODER_RESOLUTION * 0.8) {
      // Low to high rollover
      encoderRotations--;
    }
    currentPosition = ABSOLUTE_ENCODER_RESOLUTION *
                          (encoderRotations - zeroPositionRotations) +
                      currentEncoderDutyCycle - zeroPositionDutyCycle;
    encoderLastDutyCycle = currentEncoderDutyCycle;
#endif

    // Read limit switches.
    limitA = HAL_GPIO_ReadPin(LIM_A_GPIO_Port, LIM_A_Pin) == GPIO_PIN_SET;
    limitB = HAL_GPIO_ReadPin(LIM_A_GPIO_Port, LIM_A_Pin) == GPIO_PIN_SET;

    if (limitA) {
#ifdef QUADRATURE_ENCODER
      zeroPosition = htim2.Instance->CNT - limitAPosition;
#else
      zeroPositionRotations =
          encoderRotations - limitAPosition / ABSOLUTE_ENCODER_RESOLUTION;
      zeroPositionDutyCycle =
          encoderDutyCycle - limitAPosition % ABSOLUTE_ENCODER_RESOLUTION;
#endif
      controlMode = CONTROL_MODE_STOP;
    }

    if (limitB) {
#ifdef QUADRATURE_ENCODER
      zeroPosition = htim2.Instance->CNT - limitBPosition;
#else
      zeroPositionRotations =
          encoderRotations - limitBPosition / ABSOLUTE_ENCODER_RESOLUTION;
      zeroPositionDutyCycle =
          encoderDutyCycle - limitBPosition % ABSOLUTE_ENCODER_RESOLUTION;
#endif
      controlMode = CONTROL_MODE_STOP;
    }

    softLimitA = currentPosition <= softLimitAPosition;
    softLimitB = currentPosition >= softLimitBPosition;

    currentVelocity =
        (double)(currentPosition - lastPosition) / (double)deltaT * 1000;
    currentCurrent = HAL_ADC_GetValue(&hadc) & 0x00FF;

    double error;
    switch (controlMode) {
    case CONTROL_MODE_OPEN_LOOP:
      lowPass((double)target / 32768.0, &pwm, alpha, deltaT);
      break;
    case CONTROL_MODE_POSITION:
      error = target - currentPosition;
      lowPass(
          pide(error, P, I, D, errorGain, &lastError, &integralError, deltaT),
          &pwm, alpha, deltaT);
      break;
    case CONTROL_MODE_VELOCITY:
      error = target - currentVelocity;
      lowPass(
          pide(error, P, I, D, errorGain, &lastError, &integralError, deltaT),
          &pwm, alpha, deltaT);
      break;
    case CONTROL_MODE_CURRENT:
      error = target - currentCurrent;
      lowPass(
          pide(error, P, I, D, errorGain, &lastError, &integralError, deltaT),
          &pwm, alpha, deltaT);
      break;
    default:
      break;
    }

    bool parametersOK = true;
    missingParameters = 0;
    if (!(setParameters & (1 << MESSAGE_ID_SMOOTHING)) &&
        (controlMode == CONTROL_MODE_OPEN_LOOP ||
         controlMode == CONTROL_MODE_POSITION ||
         controlMode == CONTROL_MODE_VELOCITY ||
         controlMode == CONTROL_MODE_CURRENT)) {
      parametersOK = false;
      missingParameters |= 1 << MESSAGE_ID_SMOOTHING;
    }
    if (!(setParameters & (1 << MESSAGE_ID_PID)) &&
        (controlMode == CONTROL_MODE_POSITION ||
         controlMode == CONTROL_MODE_VELOCITY ||
         controlMode == CONTROL_MODE_CURRENT)) {
      parametersOK = false;
      missingParameters |= 1 << MESSAGE_ID_PID;
    }
    if (!(setParameters & (1 << MESSAGE_ID_LIMIT)) && !ignoreLimit &&
        (controlMode == CONTROL_MODE_OPEN_LOOP ||
         controlMode == CONTROL_MODE_POSITION ||
         controlMode == CONTROL_MODE_VELOCITY ||
         controlMode == CONTROL_MODE_CURRENT)) {
      parametersOK = false;
      missingParameters |= 1 << MESSAGE_ID_LIMIT;
    }
    if (!(setParameters & (1 << MESSAGE_ID_SOFT_LIMIT)) && !ignoreLimit &&
        (controlMode == CONTROL_MODE_OPEN_LOOP ||
         controlMode == CONTROL_MODE_POSITION ||
         controlMode == CONTROL_MODE_VELOCITY ||
         controlMode == CONTROL_MODE_CURRENT)) {
      parametersOK = false;
      missingParameters |= 1 << MESSAGE_ID_SOFT_LIMIT;
    }

    if (parametersOK) {
      switch (controlMode) {
      case CONTROL_MODE_STOP:
        motorA = false;
        motorB = false;
        motorPWM = 0;
        break;
      case CONTROL_MODE_OPEN_LOOP:
      case CONTROL_MODE_POSITION:
      case CONTROL_MODE_VELOCITY:
      case CONTROL_MODE_CURRENT:
        if (pwm < -0.99999)
          pwm = -0.99999;
        if (pwm > 0.99999)
          pwm = 0.99999;
        if (pwm < -0.0001) {
          if (ignoreLimit || !(limitA || softLimitA)) {
            motorA = false;
            motorB = true;
            motorPWM = -pwm * 65535;
          } else {
            motorA = false;
            motorB = false;
            motorPWM = 0;
          }
        } else if (pwm > 0.0001) {
          if (ignoreLimit || !(limitB || softLimitB)) {
            motorA = true;
            motorB = false;
            motorPWM = pwm * 65535;
          } else {
            motorA = false;
            motorB = false;
            motorPWM = 0;
          }
        } else {
          motorA = false;
          motorB = false;
          motorPWM = 0;
        }
        break;
      case CONTROL_MODE_CALIBRATING:
        break;
      default:
        break;
      }
    } else {
      // Missing at least one parameter, cycle through each
      motorA = false;
      motorB = false;
      motorPWM = 0;

      if (currentHALTick > nextParameterRequestTime &&
          HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
        if (missingParameters & (1 << nextMissingParameterRequestID)) {
          // The next missing parameter to request is still missing
        } else {
          // The next missing parameter to request isn't missing. Find the next
          // one
          nextMissingParameterRequestID =
              (nextMissingParameterRequestID + 1) % 16;
          // WARNING: this loop will be infinite if there are no set bits in
          // missingParameters. missingParameters will have a set bit if
          // parametersOK is false
          while (!(missingParameters & (1 << nextMissingParameterRequestID)))
            nextMissingParameterRequestID =
                (nextMissingParameterRequestID + 1) % 16;
        }

        nextParameterRequestTime = currentHALTick + PARAMETER_REQUEST_INTERVAL;
        CAN_TxHeaderTypeDef txHeader = {.StdId = MOCO_ID |
                                                 nextMissingParameterRequestID,
                                        .IDE = CAN_ID_STD,
                                        .RTR = CAN_RTR_REMOTE,
                                        .DLC = 0,
                                        .TransmitGlobalTime = DISABLE};
        uint8_t txData[8] = {0};

        // Cycle through the missing parameters.
        nextMissingParameterRequestID += 1;

        uint32_t txMailbox;
        HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
      }
    }

    // Output motor IN_A, IN_B, PWM
    HAL_GPIO_WritePin(IN_A_GPIO_Port, IN_A_Pin,
                      motorA ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN_B_GPIO_Port, IN_B_Pin,
                      motorB ? GPIO_PIN_SET : GPIO_PIN_RESET);
    htim14.Instance->CCR1 = motorPWM;

    // Send telemetry
    if (currentHALTick > nextReportTime &&
        HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
      nextReportTime = currentHALTick + TELEMETRY_INTERVAL;
      CAN_TxHeaderTypeDef txHeader = {.StdId = MOCO_ID | MESSAGE_ID_POSITION,
                                      .IDE = CAN_ID_STD,
                                      .RTR = CAN_RTR_DATA,
                                      .DLC = 8,
                                      .TransmitGlobalTime = DISABLE};
      uint8_t txData[8] = {
          currentPosition >> 24,
          (currentPosition >> 16) & 0xff,
          (currentPosition >> 8) & 0xff,
          currentPosition & 0xff,
          (int16_t)currentVelocity >> 8,
          (int16_t)currentVelocity & 0xff,
          currentCurrent >> 4, // TODO: Current (A/8 u8)
          (limitA ? 0b10000000 : 0) | (limitB ? 0b01000000 : 0) |
              (softLimitA ? 0b00100000 : 0) | (softLimitB ? 0b00010000 : 0) |
              (fault ? 0b00001000 : 0),
      };
      uint32_t txMailbox;
      HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data
   * Alignment and number of conversion)
   */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_4TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_7TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */
#ifdef QUADRATURE_ENCODER
  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
#else
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
#endif
  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void) {

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 16 - 1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IN_B_GPIO_Port, IN_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IN_A_GPIO_Port, IN_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_STATUS_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIAG_A_Pin */
  GPIO_InitStruct.Pin = DIAG_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIAG_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIM_A_Pin LIM_B_Pin */
  GPIO_InitStruct.Pin = LIM_A_Pin | LIM_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_B_Pin */
  GPIO_InitStruct.Pin = IN_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IN_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_A_Pin */
  GPIO_InitStruct.Pin = IN_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IN_A_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
    uint32_t cycleTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    if (cycleTime != 0) {
      encoderDutyCycle = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) *
                         ABSOLUTE_ENCODER_RESOLUTION / cycleTime;
    }
  }
}

static void reportCommandError(uint8_t commandID) {
  CAN_TxHeaderTypeDef txHeader = {.StdId = MOCO_ID | 0xD,
                                  .IDE = CAN_ID_STD,
                                  .RTR = CAN_RTR_DATA,
                                  .DLC = 1,
                                  .TransmitGlobalTime = DISABLE};
  uint8_t txData[8] = {commandID, 0, 0, 0, 0, 0, 0, 0};
  uint32_t txMailbox = 0;
  HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
}

static double pide(double error, double P, double I, double D, double errorGain,
                   double *lastError, double *integralError, uint32_t deltaT) {
  *integralError += error * deltaT;
  double out =
      P * error + I * *integralError + D * (error - *lastError) / deltaT;
  *lastError = error;
  return out;
}

static void lowPass(double in, double *out, double alpha, double deltaT) {
  *out += alpha * deltaT * (in - *out);
}

uint16_t u16FromBytes(uint8_t *bytes) { return bytes[0] << 8 | bytes[1]; }
int16_t i16FromBytes(uint8_t *bytes) { return bytes[0] << 8 | bytes[1]; }
uint32_t u32FromBytes(uint8_t *bytes) {
  return bytes[0] << 24 | bytes[1] << 16 | bytes[2] << 8 | bytes[3];
}
int32_t i32FromBytes(uint8_t *bytes) {
  return bytes[0] << 24 | bytes[1] << 16 | bytes[2] << 8 | bytes[3];
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return
   * state */
  __disable_irq();
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
