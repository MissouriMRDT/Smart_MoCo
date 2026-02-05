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
#include "adc.h"
#include "can.h"
#include "gpio.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <machine/endian.h>
#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union {
  struct __attribute__((__packed__)) {
    int32_t position;
    int16_t velocity;
    uint8_t current;
    uint8_t flags;
  } position;
  struct __attribute__((__packed__)) {
    uint8_t commandID;
  } commandError;
  struct __attribute__((__packed__)) {
    uint64_t payload;
  } echoReply;
  struct __attribute__((__packed__)) {
    uint8_t sid;
    union {
      struct __attribute__((__packed__)) {
        int16_t dutyCycle;
      } openLoop;
      struct __attribute__((__packed__)) {
        uint16_t errorGain;
        int32_t position;
      } targetPosition;
      struct __attribute__((__packed__)) {
        uint16_t errorGain;
        int32_t velocity;
      } targetVelocity;
      struct __attribute__((__packed__)) {
        uint16_t errorGain;
        int16_t current;
      } targetCurrent;
    };
  } target;
  struct __attribute__((__packed__)) {
    double rampRate;
  } setRampRate;
  struct __attribute__((__packed__)) {
    uint16_t p;
    uint16_t i;
    uint16_t d;
  } setPID;
  struct __attribute__((__packed__)) {
    int32_t aPosition;
    int32_t bPosition;
  } setSoftLimitPosition;
  struct __attribute__((__packed__)) {
    int16_t dutyCycle;
    int32_t limitSwitchPosition;
  } startPositionCalibration;
  struct __attribute__((__packed__)) {
    bool enable;
  } debugTelemetry;
  struct __attribute__((__packed__)) {
    uint64_t payload;
  } echoRequest;
} CANMessage;
typedef struct __attribute__((__packed__)) {
  uint64_t tick;
  int64_t position;
  double velocity;
  double current;
  double pOut;
  double iOut;
  double dOut;
  double error;
  double deltaT;
} DebugTelemetry;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint64_t tick = 0;           // 0.5us
volatile int32_t currentPosition = 0; // step
int32_t encoderOffset = 0;            // step

#ifndef QUADRATURE_ENCODER
// ABSOLUTE_ENCODER_RESOLUTION * step
volatile int32_t absoluteEncoderRotations = 0;
bool absoluteEncoderFirstReading = true;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void reportCommandError(uint8_t commandID);
static double pide(double error, double P, double I, double D, double errorGain,
                   double *lastError, double *integralError, double deltaT,
                   DebugTelemetry *debugTelemetry);
static void ramp(double in, double *out, double rampRate, double deltaT,
                 DebugTelemetry *debugTelemetry);
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
  // Motor PWM
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);

  // Encoder
#ifdef QUADRATURE_ENCODER
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim2, UINT16_MAX);
  encoderOffset = -UINT16_MAX;
#else
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
#endif

  // Elapsed time counter
  HAL_TIM_Base_Start_IT(&htim1);

  CAN_FilterTypeDef filter = {.FilterIdHigh = (MOCO_ID << 4) << 5,
                              .FilterIdLow = 0,
                              .FilterMaskIdHigh = 0xFFFFF0 << 5,
                              .FilterMaskIdLow = 0,
                              .FilterFIFOAssignment = CAN_FILTER_FIFO0,
                              .FilterBank = 0,
                              .FilterMode = CAN_FILTERMODE_IDMASK,
                              .FilterScale = CAN_FILTERSCALE_32BIT,
                              .FilterActivation = CAN_FILTER_ENABLE};
  HAL_CAN_ConfigFilter(&hcan, &filter);
  HAL_CAN_Start(&hcan);
  CAN_RxHeaderTypeDef rxHeader;
  CANMessage rxData = {0};

  // Controller state
  bool debugTelemetryEnabled = false;
  uint8_t debugTelemetrySending = UINT8_MAX;
  uint32_t nextDebugTelemetryCaptureTime = 0; // ms
  DebugTelemetry debugTelemetry = {0};
  ControlMode controlMode = CONTROL_MODE_STOP;
  bool ignoreLimit = false;
  uint32_t nextReportTime = 0; // ms
  bool softLimitA = false;
  bool softLimitB = false;
  uint32_t lastTick = 0;
  double deltaT = 0; // (s)
  uint32_t statusOffTime = UINT32_MAX;

  double lastError = 0;     // (target)
  double integralError = 0; // (target * s)
  double pwm = 0;           // (duty cycle) [-1.0, 1.0]

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
  double currentVelocity = 0;  // step/s
  uint16_t currentCurrent = 0; // ADC
  bool limitA = false;
  bool limitB = false;

  // Controller configuration
  double P = 0.0;
  double I = 0.0;
  double D = 0.0;
  double errorGain = 1.0;
  double rampRate = 10;                   // (duty cycle/s)
  int32_t limitSwitchPosition = 0;        // step
  int32_t softLimitAPosition = INT32_MIN; // step
  int32_t softLimitBPosition = INT32_MAX; // step

  // Controller input
  // Open Loop: duty cycle/32768 i16
  // Position: step
  // Velocity: step/s
  // Current: ADC
  int32_t target = 0;

  // Controller output
  bool motorA = false;
  bool motorB = false;
  uint16_t motorPWM = 0; // 1/65536
  double currentPositionLP = 0;
  double lastPositionLP = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    // Timeout status LED.
    if (statusOffTime < uwTick) {
      statusOffTime = UINT32_MAX;
      HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
    }

    // Process a single received CAN message
    if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0 &&
        HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader,
                             (uint8_t *)&rxData) == HAL_OK) {
      bool commandOK = true;
      if (rxHeader.RTR == CAN_RTR_DATA) {
        switch (rxHeader.StdId & 0x00F) {
        case MESSAGE_ID_TARGET:
          if (rxHeader.DLC < 1) {
            commandOK = false;
            break;
          }
          switch (rxData.target.sid & 0b11111110) {
          case MESSAGE_SID_OPEN_LOOP:
            if (rxHeader.DLC < 3) {
              commandOK = false;
              break;
            }
            ignoreLimit = rxData.target.sid & 0b00000001;
            controlMode = CONTROL_MODE_OPEN_LOOP;
            target = rxData.target.openLoop.dutyCycle;
            break;
          case MESSAGE_SID_POSITION:
            if (rxHeader.DLC < 7) {
              commandOK = false;
              break;
            }
            if (controlMode != CONTROL_MODE_POSITION) {
              lastError = 0;
              integralError = 0;
              if (lastPIDUsed)
                setParameters &= ~(1 << MESSAGE_ID_PID);
            }
            lastPIDUsed = true;
            controlMode = CONTROL_MODE_POSITION;
            ignoreLimit = rxData.target.sid & 0b00000001;
            errorGain = (double)rxData.target.targetPosition.errorGain / 1024;
            target = rxData.target.targetPosition.position;
            break;
          case MESSAGE_SID_VELOCITY:
            if (rxHeader.DLC < 7) {
              commandOK = false;
              break;
            }
            if (controlMode != CONTROL_MODE_VELOCITY) {
              lastError = 0;
              integralError = 0;
              if (lastPIDUsed)
                setParameters &= ~(1 << MESSAGE_ID_PID);
            }
            lastPIDUsed = true;
            controlMode = CONTROL_MODE_VELOCITY;
            ignoreLimit = rxData.target.sid & 0b00000001;
            errorGain = (double)rxData.target.targetVelocity.errorGain / 1024;
            target = rxData.target.targetVelocity.velocity;
            break;
          case MESSAGE_SID_CURRENT:
            if (rxHeader.DLC < 5) {
              commandOK = false;
              break;
            }
            if (controlMode != CONTROL_MODE_CURRENT) {
              lastError = 0;
              integralError = 0;
              if (lastPIDUsed)
                setParameters &= ~(1 << MESSAGE_ID_PID);
            }
            lastPIDUsed = true;
            controlMode = CONTROL_MODE_CURRENT;
            ignoreLimit = rxData.target.sid & 0b00000001;
            errorGain = (double)rxData.target.targetCurrent.errorGain / 1024;
            target = rxData.target.targetCurrent.current;
            break;
          default:
            break;
          }
          break;
        case MESSAGE_ID_SMOOTHING:
          if (rxHeader.DLC < 8) {
            commandOK = false;
            break;
          }
          if (rxData.setRampRate.rampRate < 0.1) {
            commandOK = false;
            break;
          }
          rampRate = rxData.setRampRate.rampRate;
          setParameters |= 1 << MESSAGE_ID_SMOOTHING;
          break;
        case MESSAGE_ID_PID:
          if (rxHeader.DLC < 6) {
            commandOK = false;
            break;
          }
          if (lastPIDUsed)
            lastPIDUsed = false;
          P = (double)rxData.setPID.p / 256;
          I = (double)rxData.setPID.i / 256;
          D = (double)rxData.setPID.d / 256;
          setParameters |= 1 << MESSAGE_ID_PID;
          break;
        case MESSAGE_ID_SOFT_LIMIT:
          if (rxHeader.DLC < 8) {
            commandOK = false;
            break;
          }
          int32_t a1 = rxData.setSoftLimitPosition.aPosition;
          int32_t b1 = rxData.setSoftLimitPosition.bPosition;
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
          if (rxHeader.DLC < 6) {
            commandOK = false;
            break;
          }
          controlMode = CONTROL_MODE_CALIBRATING;
          target = rxData.startPositionCalibration.dutyCycle;
          limitSwitchPosition =
              rxData.startPositionCalibration.limitSwitchPosition;
          break;
        case MESSAGE_ID_DEBUG:
          if (rxHeader.DLC < 1) {
            commandOK = false;
            break;
          }
          debugTelemetryEnabled = rxData.debugTelemetry.enable;
          break;
        case MESSAGE_ID_STOP:
          controlMode = CONTROL_MODE_STOP;
          P = 0.0;
          I = 0.0;
          D = 0.0;
          errorGain = 1.0;
          rampRate = 10;
          pwm = 0;
          target = 0;
          lastError = 0;
          integralError = 0;
          lastPIDUsed = false;
          setParameters = 0;
          break;
        case MESSAGE_ID_ECHO_REQUEST: {
          HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
          statusOffTime = uwTick + 100;
          CAN_TxHeaderTypeDef txHeader = {.StdId = (MOCO_ID << 4) |
                                                   MESSAGE_ID_ECHO_REPLY,
                                          .IDE = CAN_ID_STD,
                                          .RTR = CAN_RTR_DATA,
                                          .DLC = rxHeader.DLC,
                                          .TransmitGlobalTime = DISABLE};
          uint32_t txMailbox;
          HAL_CAN_AddTxMessage(&hcan, &txHeader,
                               (uint8_t *)&rxData.echoRequest.payload,
                               &txMailbox);
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
    uint64_t currentTick = tick + __HAL_TIM_GET_COUNTER(&htim1); // 0.5 us
    if (currentTick < lastTick) {
      // Timer overflowed since the last iteration and tick hasn't been
      // incremented by UINT16_MAX yet.
      currentTick += UINT16_MAX;
    }
    deltaT = (double)(currentTick - lastTick) / 2000000;
    lastTick = currentTick;

    currentPositionLP = lastPositionLP;
    lowPass((double)currentPosition, &currentPositionLP, 20.0, deltaT);
    currentVelocity = (double)(currentPositionLP - lastPositionLP) / deltaT;
    lastPositionLP = currentPositionLP;

#ifdef QUADRATURE_ENCODER
    currentPosition = __HAL_TIM_GET_COUNTER(&htim2) + encoderOffset;
#endif
    // Absolute encoder updates currentPosition from HAL_TIM_IC_CaptureCallback.

    // Read limit switches.
    limitA = HAL_GPIO_ReadPin(LIM_A_GPIO_Port, LIM_A_Pin) == GPIO_PIN_SET;
    limitB = HAL_GPIO_ReadPin(LIM_B_GPIO_Port, LIM_B_Pin) == GPIO_PIN_SET;

    softLimitA = currentPosition <= softLimitAPosition;
    softLimitB = currentPosition >= softLimitBPosition;

    currentCurrent = HAL_ADC_GetValue(&hadc) & 0x00FF;

    DebugTelemetry *debugTelemetryCapture =
        debugTelemetryEnabled && uwTick > nextDebugTelemetryCaptureTime
            ? &debugTelemetry
            : NULL;
    if (debugTelemetryCapture != NULL) {
      debugTelemetry.tick = currentTick;
      debugTelemetry.position = currentPosition;
      debugTelemetry.velocity = currentVelocity;
      debugTelemetry.current = currentCurrent;
    }

    double error;
    switch (controlMode) {
    case CONTROL_MODE_OPEN_LOOP:
      pwm = pwm < -1 ? -1 : pwm > 1 ? 1 : pwm;
      ramp((double)target / 32768, &pwm, rampRate, deltaT,
           debugTelemetryCapture);
      break;
    case CONTROL_MODE_POSITION:
      error = target - currentPosition;
      pwm = pide(error, P, I, D, errorGain, &lastError, &integralError, deltaT,
                 debugTelemetryCapture);
      break;
    case CONTROL_MODE_VELOCITY:
      error = target - currentVelocity;
      pwm = pide(error, P, I, D, errorGain, &lastError, &integralError, deltaT,
                 debugTelemetryCapture);
      break;
    case CONTROL_MODE_CURRENT:
      error = target - currentCurrent;
      pwm = pide(error, P, I, D, errorGain, &lastError, &integralError, deltaT,
                 debugTelemetryCapture);
      break;
    default:
      break;
    }

    bool parametersOK = true;
    missingParameters = 0;
    if (!(setParameters & (1 << MESSAGE_ID_SMOOTHING)) &&
        (controlMode == CONTROL_MODE_OPEN_LOOP)) {
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
    if (!(setParameters & (1 << MESSAGE_ID_SOFT_LIMIT)) &&
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
          if (!softLimitA && (ignoreLimit || !limitA)) {
            motorA = false;
            motorB = true;
            motorPWM = -pwm * UINT16_MAX;
          } else {
            motorA = false;
            motorB = false;
            motorPWM = 0;
          }
        } else if (pwm > 0.0001) {
          if (!softLimitB && (ignoreLimit || !limitB)) {
            motorA = true;
            motorB = false;
            motorPWM = pwm * UINT16_MAX;
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
        if (target < 0 ? limitA : limitB) {
          encoderOffset += limitSwitchPosition - currentPosition;
          controlMode = CONTROL_MODE_STOP;
          CAN_TxHeaderTypeDef txHeader = {
              .StdId = (MOCO_ID << 4) | MESSAGE_ID_POSITION_CALIBRATED,
              .IDE = CAN_ID_STD,
              .RTR = CAN_RTR_DATA,
              .DLC = 0,
              .TransmitGlobalTime = DISABLE};
          uint8_t txData[8] = {0};
          uint32_t txMailbox;
          HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
        } else if (target < -1) {
          motorA = false;
          motorB = true;
          motorPWM = -target / 2;
        } else if (target > 1) {
          motorA = false;
          motorB = true;
          motorPWM = target / 2;
        } else {
          motorA = false;
          motorB = false;
          motorPWM = 0;
        }
        break;
      default:
        break;
      }
    } else {
      // Missing at least one parameter, cycle through each
      motorA = false;
      motorB = false;
      motorPWM = 0;

      if (uwTick > nextParameterRequestTime &&
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

        nextParameterRequestTime = uwTick + PARAMETER_REQUEST_INTERVAL;
        CAN_TxHeaderTypeDef txHeader = {.StdId = (MOCO_ID << 4) |
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
    if (uwTick > nextReportTime && HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
      nextReportTime = uwTick + TELEMETRY_INTERVAL;
      CAN_TxHeaderTypeDef txHeader = {.StdId =
                                          (MOCO_ID << 4) | MESSAGE_ID_POSITION,
                                      .IDE = CAN_ID_STD,
                                      .RTR = CAN_RTR_DATA,
                                      .DLC = 8,
                                      .TransmitGlobalTime = DISABLE};

      CANMessage txData = {
          .position = {.position = currentPosition,
                       .velocity = currentVelocity,
                       .current = currentCurrent >> 4, // TODO: Current (A/8 u8)
                       .flags = (limitA ? 0b10000000 : 0) |
                                (limitB ? 0b01000000 : 0) |
                                (softLimitA ? 0b00100000 : 0) |
                                (softLimitB ? 0b00010000 : 0)}};
      uint32_t txMailbox;
      HAL_CAN_AddTxMessage(&hcan, &txHeader, (uint8_t *)&txData, &txMailbox);
    }

    if (debugTelemetryEnabled) {
      if (uwTick > nextDebugTelemetryCaptureTime) {
        nextDebugTelemetryCaptureTime = uwTick + DEBUG_TELEMETRY_INTERVAL;
        debugTelemetrySending = 0;
      }
      if (debugTelemetrySending < 9 &&
          HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
        CAN_TxHeaderTypeDef txHeader = {.StdId = MESSAGE_ID_DEBUG_OFFSET |
                                                 debugTelemetrySending,
                                        .IDE = CAN_ID_STD,
                                        .RTR = CAN_RTR_DATA,
                                        .DLC = 8,
                                        .TransmitGlobalTime = DISABLE};

        uint32_t txMailbox;
        if (HAL_CAN_AddTxMessage(&hcan, &txHeader,
                                 (uint8_t *)&debugTelemetry +
                                     debugTelemetrySending * 8,
                                 &txMailbox) == HAL_OK)
          debugTelemetrySending++;
      }
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

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1)
    tick += UINT16_MAX;
}

#ifndef QUADRATURE_ENCODER
uint32_t encoderLastWidth = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim != &htim2)
    return;
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    uint32_t encoderPeriod = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    if (encoderPeriod) {
      // `if (encoderPeriod)` ensures we don't divide by 0.
      uint32_t encoderWidth = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
      if (encoderLastWidth) {
        // Don't rollover on startup.
        if (encoderLastWidth > encoderPeriod * 0.8 &&
            encoderWidth < encoderPeriod * 0.2) {
          // High to low rollover:
          absoluteEncoderRotations++;
        } else if (encoderLastWidth < encoderPeriod * 0.2 &&
                   encoderWidth > encoderPeriod * 0.8) {
          // Low to high rollover:
          absoluteEncoderRotations--;
        }
      }
#ifdef ABSOLUTE_ENCODER_STARTUP_THRESHOLD
      else {
        // On startup, if the encoder position is past
        // ABSOLUTE_ENCODER_STARTUP_THRESHOLD, move the position back one
        // revolution.
        if (ABSOLUTE_ENCODER_RESOLUTION * encoderWidth / encoderPeriod >
            ABSOLUTE_ENCODER_STARTUP_THRESHOLD)
          absoluteEncoderRotations = -1;
      }
#endif
      encoderLastWidth = encoderWidth;
      currentPosition =
          ABSOLUTE_ENCODER_RESOLUTION * absoluteEncoderRotations +
          ABSOLUTE_ENCODER_RESOLUTION * encoderWidth / encoderPeriod +
          encoderOffset;
    }
  }
}
#endif

static void reportCommandError(uint8_t commandID) {
  CAN_TxHeaderTypeDef txHeader = {.StdId = (MOCO_ID << 4) | MESSAGE_ID_ERROR,
                                  .IDE = CAN_ID_STD,
                                  .RTR = CAN_RTR_DATA,
                                  .DLC = 1,
                                  .TransmitGlobalTime = DISABLE};
  CANMessage txData = {.commandError = {commandID}};
  uint32_t txMailbox = 0;
  HAL_CAN_AddTxMessage(&hcan, &txHeader, (uint8_t *)&txData, &txMailbox);
}

static double pide(double error, double P, double I, double D, double errorGain,
                   double *lastError, double *integralError, double deltaT,
                   DebugTelemetry *debugTelemetry) {
  error *= errorGain;
  *integralError += error * deltaT;
  if (debugTelemetry != NULL) {
    debugTelemetry->pOut = P * error;
    debugTelemetry->iOut = I * *integralError;
    debugTelemetry->dOut = D * (error - *lastError) * deltaT;
    debugTelemetry->error = error;
    debugTelemetry->deltaT = deltaT;
  }
  double out =
      P * error + I * *integralError + D * (error - *lastError) * deltaT;
  *lastError = error;
  return out;
}

static void ramp(double in, double *out, double rampRate, double deltaT,
                 DebugTelemetry *debugTelemetry) {
  if (debugTelemetry != NULL) {
    debugTelemetry->pOut = *out;
    debugTelemetry->iOut = 0;
    debugTelemetry->dOut = 0;
    debugTelemetry->error = in - *out;
    debugTelemetry->deltaT = deltaT;
  }
  if (in > *out + rampRate * deltaT) {
    *out += rampRate * deltaT;
  } else if (in < *out - rampRate * deltaT) {
    *out -= rampRate * deltaT;
  } else {
    *out = in;
  }
}

static void lowPass(double in, double *out, double alpha, double deltaT) {
  *out += alpha * deltaT * (in - *out);
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
  while (1) {
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
    while (__HAL_TIM_GET_COUNTER(&htim1) < 0x8000)
      ;
    while (__HAL_TIM_GET_COUNTER(&htim1) >= 0x8000)
      ;
    while (__HAL_TIM_GET_COUNTER(&htim1) < 0x8000)
      ;
    while (__HAL_TIM_GET_COUNTER(&htim1) >= 0x8000)
      ;
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
    while (__HAL_TIM_GET_COUNTER(&htim1) < 0x8000)
      ;
    while (__HAL_TIM_GET_COUNTER(&htim1) >= 0x8000)
      ;
    while (__HAL_TIM_GET_COUNTER(&htim1) < 0x8000)
      ;
    while (__HAL_TIM_GET_COUNTER(&htim1) >= 0x8000)
      ;
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
