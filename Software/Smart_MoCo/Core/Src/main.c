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
#include "gpio.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
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
    float rampRate;
  } setRampRate;
  struct __attribute__((__packed__)) {
    float p;
    float i;
  } setPI;
  struct __attribute__((__packed__)) {
    float d;
  } setD;
  struct __attribute__((__packed__)) {
    int32_t aPosition;
    int32_t bPosition;
  } setSoftLimitPosition;
  struct __attribute__((__packed__)) {
    uint8_t ab;
  } ignoreLimitSwitch;
  struct __attribute__((__packed__)) {
    int16_t dutyCycle;
    int32_t limitSwitchPosition;
  } startPositionCalibration;
  struct __attribute__((__packed__)) {
    bool enable;
  } debugTelemetry;
  struct __attribute__((__packed__)) {
    int16_t fwdMax;
    int16_t fwdMin;
    int16_t revMin;
    int16_t revMax;
  } setDutyCycleRange;
  struct __attribute__((__packed__)) {
    uint64_t payload;
  } echoRequest;
  struct __attribute__((__packed__)) {
    int16_t dutyCycle;
  } openLoop;
  struct __attribute__((__packed__)) {
    int16_t feedForward;
    int32_t position;
  } targetPosition;
  struct __attribute__((__packed__)) {
    int16_t feedForward;
    float velocity;
  } targetVelocity;
  struct __attribute__((__packed__)) {
    int16_t feedForward;
    int16_t current;
  } targetCurrent;
} CANMessage;
typedef struct __attribute__((__packed__)) {
  uint32_t tick;
  int32_t position;
  float velocity;
  float current;
  float pOut;
  float iOut;
  float dOut;
  float error;
  float deltaT;
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

volatile uint32_t tick = 0;           // 0.5us
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
static float pide(float error, float P, float I, float D, float feedForward,
                  float *lastError, float *integralError, float deltaT,
                  DebugTelemetry *debugTelemetry);
static void ramp(float in, float *out, float rampRate, float deltaT,
                 DebugTelemetry *debugTelemetry);
static void lowPass(float in, float *out, float alpha, float deltaT);
static int16_t clamp4(int16_t x, int16_t pMax, int16_t pMin, int16_t nMin,
                      int16_t nMax);
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
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  LL_SYSCFG_EnablePinRemap();

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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // Motor PWM
  LL_TIM_EnableCounter(TIM14);
  LL_TIM_CC_EnableChannel(TIM14, LL_TIM_CHANNEL_CH1);

// Encoder
#ifdef QUADRATURE_ENCODER
  LL_TIM_EnableCounter(TIM2);
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
  LL_TIM_SetCounter(TIM2, UINT16_MAX);
  encoderOffset = -UINT16_MAX;
#else
  LL_TIM_EnableIT_CC1(TIM2);
  LL_TIM_EnableIT_CC2(TIM2);
#endif

  // Elapsed time counter
  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_EnableCounter(TIM1);

  // STID[10:5] == MOCO_ID
  CAN_ConfigFilter(0, true, false, false, true, (MOCO_ID << 6) << 21,
                   0xFE000000);

  CAN_RxHeader rxHeader;
  CANMessage rxData = {0};

  // Controller state
  bool debugTelemetryEnabled = false;
  uint8_t debugTelemetrySending = UINT8_MAX;
  uint32_t nextDebugTelemetryCaptureTime = 0; // ms
  DebugTelemetry debugTelemetry = {0};
  ControlMode controlMode = CONTROL_MODE_STOP;

  bool ignoreLimitA = false;
  bool ignoreLimitB = false;
  bool softLimitA = false;
  bool softLimitB = false;
  int16_t fwdMaxPwm = INT16_MIN;
  int16_t fwdMinPwm = 0;
  int16_t revMinPwm = 0;
  int16_t revMaxPwm = INT16_MIN;

  uint32_t nextReportTime = 0; // ms
  uint32_t lastTick = 0;
  float deltaT = 0; // (s)
  uint32_t statusOffTime = UINT32_MAX;

  float lastError = 0;     // (target)
  float integralError = 0; // (target * s)
  float pwm = 0;           // (duty cycle) [-1.0, 1.0]

  // Parameter tracking
  uint64_t setParameters = 0;
  bool lastPIDUsed = false;
  uint64_t missingParameters = 0;
  uint16_t nextMissingParameterRequestID = 0;
  uint32_t nextParameterRequestTime = 0;
  // Set PID logic
  // +-----+      +---+         +----+
  // |Unset|-PID->|Set|-Target->|Used|
  // +-----+      +---+         +----+
  //   ^ ^--Reset--/ ^----PID----/ |
  //   \---Reset OR Other Target---/

  // Controller feedback
  float currentVelocity = 0;   // step/s
  uint16_t currentCurrent = 0; // ADC
  bool limitA = false;
  bool limitB = false;

  // Controller configuration
  float P = 0.0;
  float I = 0.0;
  float D = 0.0;
  int16_t feedForward = 0;
  float rampRate = 10;                    // (duty cycle/s)
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
  float currentPositionLP = 0;
  float lastPositionLP = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    // Timeout status LED.
    if (statusOffTime < tick) {
      statusOffTime = UINT32_MAX;
      LL_GPIO_ResetOutputPin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
    }

    // Process a single received CAN message
    if (CAN_GetRxMessage(CAN_RX_FIFO0, &rxHeader, (uint8_t *)&rxData)) {
      bool commandOK = true;
      if (rxHeader.RTR == CAN_RTR_DATA) {
        switch (rxHeader.StdId & 0x03F) {
        case MESSAGE_ID_STOP:
          controlMode = CONTROL_MODE_STOP;
          P = 0.0;
          I = 0.0;
          D = 0.0;
          feedForward = 0;
          rampRate = 10;
          pwm = 0;
          target = 0;
          lastError = 0;
          integralError = 0;
          lastPIDUsed = false;
          setParameters = 0;
          break;
        case MESSAGE_ID_RAMP_RATE:
          if (rxHeader.DLC < 4) {
            commandOK = false;
            break;
          }
          rampRate = rxData.setRampRate.rampRate;
          setParameters |= 1 << MESSAGE_ID_RAMP_RATE;
          break;
        case MESSAGE_ID_PI:
          if (rxHeader.DLC < 8) {
            commandOK = false;
            break;
          }
          if (lastPIDUsed)
            lastPIDUsed = false;
          P = rxData.setPI.p;
          I = rxData.setPI.i;
          setParameters |= 1 << MESSAGE_ID_PI;
          break;
        case MESSAGE_ID_D:
          if (rxHeader.DLC < 4) {
            commandOK = false;
            break;
          }
          if (lastPIDUsed)
            lastPIDUsed = false;
          D = rxData.setD.d;
          setParameters |= 1 << MESSAGE_ID_D;
          break;
        case MESSAGE_ID_IGNORE_LIMIT:
          if (rxHeader.DLC < 1) {
            commandOK = false;
            break;
          }
          ignoreLimitA = (rxData.ignoreLimitSwitch.ab & 0b10000000) != 0;
          ignoreLimitB = (rxData.ignoreLimitSwitch.ab & 0b01000000) != 0;
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
        case MESSAGE_ID_DUTY_CYCLE_RANGE:
          if (rxHeader.DLC < 8) {
            commandOK = false;
            break;
          }
          if (rxData.setDutyCycleRange.fwdMax <=
                  rxData.setDutyCycleRange.fwdMin ||
              rxData.setDutyCycleRange.fwdMin <=
                  rxData.setDutyCycleRange.revMin ||
              rxData.setDutyCycleRange.revMin <=
                  rxData.setDutyCycleRange.revMax) {
            // Limit A must be less than limit B
            commandOK = false;
            break;
          }
          fwdMaxPwm = rxData.setDutyCycleRange.fwdMax;
          fwdMinPwm = rxData.setDutyCycleRange.fwdMin;
          revMinPwm = rxData.setDutyCycleRange.revMin;
          revMaxPwm = rxData.setDutyCycleRange.revMax;
          setParameters |= 1 << MESSAGE_ID_DUTY_CYCLE_RANGE;
          break;
        case MESSAGE_ID_ECHO_REQUEST: {
          LL_GPIO_SetOutputPin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
          statusOffTime = GetTick() + 200000;
          CAN_TxHeader txHeader = {.StdId =
                                       (MOCO_ID << 6) | MESSAGE_ID_ECHO_REPLY,
                                   .IDE = CAN_ID_STD,
                                   .RTR = CAN_RTR_DATA,
                                   .DLC = rxHeader.DLC,
                                   .TransmitGlobalTime = DISABLE};
          CAN_Tx_Mailboxes txMailbox;
          CAN_AddTxMessage(&txHeader, (uint8_t *)&rxData.echoRequest.payload,
                           &txMailbox);
          break;
        }
        case MESSAGE_ID_OPEN_LOOP:
          if (rxHeader.DLC < 2) {
            commandOK = false;
            break;
          }
          controlMode = CONTROL_MODE_OPEN_LOOP;
          target = rxData.openLoop.dutyCycle;
          break;
        case MESSAGE_ID_TARGET_POSITION:
          if (rxHeader.DLC < 7) {
            commandOK = false;
            break;
          }
          if (controlMode != CONTROL_MODE_POSITION) {
            lastError = 0;
            integralError = 0;
            if (lastPIDUsed)
              setParameters &= ~((1 << MESSAGE_ID_PI) | (1 << MESSAGE_ID_D));
          }
          lastPIDUsed = true;
          controlMode = CONTROL_MODE_POSITION;
          feedForward = rxData.targetPosition.feedForward;
          target = rxData.targetPosition.position;
          break;
        case MESSAGE_ID_TARGET_VELOCITY:
          if (rxHeader.DLC < 7) {
            commandOK = false;
            break;
          }
          if (controlMode != CONTROL_MODE_VELOCITY) {
            lastError = 0;
            integralError = 0;
            if (lastPIDUsed)
              setParameters &= ~((1 << MESSAGE_ID_PI) | (1 << MESSAGE_ID_D));
          }
          lastPIDUsed = true;
          controlMode = CONTROL_MODE_VELOCITY;
          feedForward = rxData.targetVelocity.feedForward;
          target = rxData.targetVelocity.velocity;
          break;
        case MESSAGE_ID_TARGET_CURRENT:
          if (rxHeader.DLC < 5) {
            commandOK = false;
            break;
          }
          if (controlMode != CONTROL_MODE_CURRENT) {
            lastError = 0;
            integralError = 0;
            if (lastPIDUsed)
              setParameters &= ~((1 << MESSAGE_ID_PI) | (1 << MESSAGE_ID_D));
          }
          lastPIDUsed = true;
          controlMode = CONTROL_MODE_CURRENT;
          feedForward = rxData.targetCurrent.feedForward;
          target = rxData.targetCurrent.current;
          break;
        default:
          break;
        }
      }
      if (!commandOK) {
        reportCommandError(rxHeader.StdId & 0x03F);
      }
    }

    // Update controller
    uint32_t currentTick = GetTick(); // 0.5 us
    if (currentTick < lastTick) {
      // Timer overflowed since the last iteration and tick hasn't been
      // incremented by UINT16_MAX yet.
      currentTick += UINT16_MAX;
    }
    deltaT = (float)(currentTick - lastTick) / 2000000;
    lastTick = currentTick;

    currentPositionLP = lastPositionLP;
    lowPass((float)currentPosition, &currentPositionLP, 20.0, deltaT);
    currentVelocity = (float)(currentPositionLP - lastPositionLP) / deltaT;
    lastPositionLP = currentPositionLP;

#ifdef QUADRATURE_ENCODER
    currentPosition = __HAL_TIM_GET_COUNTER(&htim2) + encoderOffset;
#endif
    // Absolute encoder updates currentPosition from HAL_TIM_IC_CaptureCallback.

    // Read limit switches.
    limitA = (LL_GPIO_ReadInputPort(LIM_A_GPIO_Port) & LIM_A_Pin) != 0;
    limitB = (LL_GPIO_ReadInputPort(LIM_B_GPIO_Port) & LIM_B_Pin) != 0;

    softLimitA = currentPosition <= softLimitAPosition;
    softLimitB = currentPosition >= softLimitBPosition;

    currentCurrent = LL_ADC_REG_ReadConversionData32(ADC1) & 0x00FF;

    DebugTelemetry *debugTelemetryCapture =
        debugTelemetryEnabled &&
                (GetTick() / TICKS_PER_MS) > nextDebugTelemetryCaptureTime
            ? &debugTelemetry
            : NULL;
    if (debugTelemetryCapture != NULL) {
      debugTelemetry.tick = currentTick;
      debugTelemetry.position = currentPosition;
      debugTelemetry.velocity = currentVelocity;
      debugTelemetry.current = currentCurrent;
    }

    float error;
    switch (controlMode) {
    case CONTROL_MODE_OPEN_LOOP:
      pwm = pwm < -1 ? -1 : pwm > 1 ? 1 : pwm;
      ramp((float)target / 32768, &pwm, rampRate, deltaT,
           debugTelemetryCapture);
      break;
    case CONTROL_MODE_POSITION:
      error = target - currentPosition;
      pwm = pide(error, P, I, D, feedForward, &lastError, &integralError,
                 deltaT, debugTelemetryCapture);
      break;
    case CONTROL_MODE_VELOCITY:
      error = target - currentVelocity;
      pwm = pide(error, P, I, D, feedForward, &lastError, &integralError,
                 deltaT, debugTelemetryCapture);
      break;
    case CONTROL_MODE_CURRENT:
      error = target - currentCurrent;
      pwm = pide(error, P, I, D, feedForward, &lastError, &integralError,
                 deltaT, debugTelemetryCapture);
      break;
    default:
      break;
    }

    bool parametersOK = true;
    missingParameters = 0;
    if (!(setParameters & (1 << MESSAGE_ID_RAMP_RATE)) &&
        (controlMode == CONTROL_MODE_OPEN_LOOP)) {
      parametersOK = false;
      missingParameters |= 1 << MESSAGE_ID_RAMP_RATE;
    }
    if (!(setParameters & (1 << MESSAGE_ID_PI)) &&
        (controlMode == CONTROL_MODE_POSITION ||
         controlMode == CONTROL_MODE_VELOCITY ||
         controlMode == CONTROL_MODE_CURRENT)) {
      parametersOK = false;
      missingParameters |= 1 << MESSAGE_ID_PI;
    }
    if (!(setParameters & (1 << MESSAGE_ID_D)) &&
        (controlMode == CONTROL_MODE_POSITION ||
         controlMode == CONTROL_MODE_VELOCITY ||
         controlMode == CONTROL_MODE_CURRENT)) {
      parametersOK = false;
      missingParameters |= 1 << MESSAGE_ID_D;
    }
    if (!(setParameters & (1 << MESSAGE_ID_IGNORE_LIMIT)) &&
        (controlMode == CONTROL_MODE_OPEN_LOOP ||
         controlMode == CONTROL_MODE_POSITION ||
         controlMode == CONTROL_MODE_VELOCITY ||
         controlMode == CONTROL_MODE_CURRENT)) {
      parametersOK = false;
      missingParameters |= 1 << MESSAGE_ID_IGNORE_LIMIT;
    }
    if (!(setParameters & (1 << MESSAGE_ID_SOFT_LIMIT)) &&
        (controlMode == CONTROL_MODE_OPEN_LOOP ||
         controlMode == CONTROL_MODE_POSITION ||
         controlMode == CONTROL_MODE_VELOCITY ||
         controlMode == CONTROL_MODE_CURRENT)) {
      parametersOK = false;
      missingParameters |= 1 << MESSAGE_ID_SOFT_LIMIT;
    }
    if (!(setParameters & (1 << MESSAGE_ID_DUTY_CYCLE_RANGE)) &&
        (controlMode == CONTROL_MODE_OPEN_LOOP ||
         controlMode == CONTROL_MODE_POSITION ||
         controlMode == CONTROL_MODE_VELOCITY ||
         controlMode == CONTROL_MODE_CURRENT)) {
      parametersOK = false;
      missingParameters |= 1 << MESSAGE_ID_DUTY_CYCLE_RANGE;
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
        int16_t clampedPWM =
            clamp4(pwm * INT16_MAX, fwdMaxPwm, fwdMinPwm, revMinPwm, revMaxPwm);
        if (clampedPWM < 0) {
          if (!softLimitA && (ignoreLimitA || !limitA)) {
            motorA = false;
            motorB = true;
            motorPWM = -clampedPWM;
          } else {
            motorA = false;
            motorB = false;
            motorPWM = 0;
          }
        } else if (clampedPWM > 0) {
          if (!softLimitB && (ignoreLimitB || !limitB)) {
            motorA = true;
            motorB = false;
            motorPWM = clampedPWM;
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
          CAN_TxHeader txHeader = {.StdId = (MOCO_ID << 6) |
                                            MESSAGE_ID_POSITION_CALIBRATED,
                                   .IDE = CAN_ID_STD,
                                   .RTR = CAN_RTR_DATA,
                                   .DLC = 0,
                                   .TransmitGlobalTime = false};
          uint8_t txData[8] = {0};
          CAN_Tx_Mailboxes txMailbox;
          CAN_AddTxMessage(&txHeader, txData, &txMailbox);
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

      if ((GetTick() / TICKS_PER_MS) > nextParameterRequestTime &&
          CAN_GetTxMailboxesFreeLevel() > 0) {
        if (missingParameters & (1 << nextMissingParameterRequestID)) {
          // The next missing parameter to request is still missing
        } else {
          // The next missing parameter to request isn't missing. Find the next
          // one
          nextMissingParameterRequestID =
              (nextMissingParameterRequestID + 1) % 64;
          // WARNING: this loop will be infinite if there are no set bits in
          // missingParameters. missingParameters will have a set bit if
          // parametersOK is false
          while (!(missingParameters & (1 << nextMissingParameterRequestID)))
            nextMissingParameterRequestID =
                (nextMissingParameterRequestID + 1) % 64;
        }

        nextParameterRequestTime =
            (GetTick() / TICKS_PER_MS) + PARAMETER_REQUEST_INTERVAL;
        CAN_TxHeader txHeader = {.StdId = (MOCO_ID << 6) |
                                          nextMissingParameterRequestID,
                                 .IDE = CAN_ID_STD,
                                 .RTR = CAN_RTR_REMOTE,
                                 .DLC = 0,
                                 .TransmitGlobalTime = false};
        uint8_t txData[8] = {0};

        // Cycle through the missing parameters.
        nextMissingParameterRequestID += 1;

        CAN_Tx_Mailboxes txMailbox;
        CAN_AddTxMessage(&txHeader, txData, &txMailbox);
      }
    }

    // Output motor IN_A, IN_B, PWM
    if (motorA)
      LL_GPIO_SetOutputPin(IN_A_GPIO_Port, IN_A_Pin);
    else
      LL_GPIO_ResetOutputPin(IN_A_GPIO_Port, IN_A_Pin);
    if (motorB)
      LL_GPIO_SetOutputPin(IN_B_GPIO_Port, IN_B_Pin);
    else
      LL_GPIO_ResetOutputPin(IN_B_GPIO_Port, IN_B_Pin);
    LL_TIM_OC_SetCompareCH1(TIM14, motorPWM);

    // Send telemetry
    if ((GetTick() / TICKS_PER_MS) > nextReportTime &&
        CAN_GetTxMailboxesFreeLevel() > 0) {
      nextReportTime = (GetTick() / TICKS_PER_MS) + TELEMETRY_INTERVAL;
      CAN_TxHeader txHeader = {.StdId = (MOCO_ID << 6) | MESSAGE_ID_POSITION,
                               .IDE = CAN_ID_STD,
                               .RTR = CAN_RTR_DATA,
                               .DLC = 8,
                               .TransmitGlobalTime = false};

      CANMessage txData = {
          .position = {.position = currentPosition,
                       .velocity = currentVelocity,
                       .current = currentCurrent >> 4, // TODO: Current (A/8 u8)
                       .flags = (limitA ? 0b10000000 : 0) |
                                (limitB ? 0b01000000 : 0) |
                                (softLimitA ? 0b00100000 : 0) |
                                (softLimitB ? 0b00010000 : 0)}};
      CAN_Tx_Mailboxes txMailbox;
      CAN_AddTxMessage(&txHeader, (uint8_t *)&txData, &txMailbox);
    }

    if (debugTelemetryEnabled) {
      if ((GetTick() / TICKS_PER_MS) > nextDebugTelemetryCaptureTime) {
        nextDebugTelemetryCaptureTime =
            (GetTick() / TICKS_PER_MS) + DEBUG_TELEMETRY_INTERVAL;
        debugTelemetrySending = 0;
      }
      if (debugTelemetrySending < 9 && CAN_GetTxMailboxesFreeLevel() > 0) {
        CAN_TxHeader txHeader = {.StdId = MESSAGE_ID_DEBUG_OFFSET |
                                          debugTelemetrySending,
                                 .IDE = CAN_ID_STD,
                                 .RTR = CAN_RTR_DATA,
                                 .DLC = 4,
                                 .TransmitGlobalTime = false};

        CAN_Tx_Mailboxes txMailbox;
        if (CAN_AddTxMessage(&txHeader,
                             (uint8_t *)&debugTelemetry +
                                 debugTelemetrySending * 4,
                             &txMailbox))
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0) {
  }
  LL_RCC_HSI_Enable();

  /* Wait till HSI is ready */
  while (LL_RCC_HSI_IsReady() != 1) {
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI14_Enable();

  /* Wait till HSI14 is ready */
  while (LL_RCC_HSI14_IsReady() != 1) {
  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

  /* Wait till System clock is ready */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
  }
  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
  LL_RCC_HSI14_EnableADCControl();
}

/* USER CODE BEGIN 4 */
// Return current time as a multiple of 0.5us.
uint32_t GetTick(void) { return tick + LL_TIM_GetCounter(TIM1); }

void TIM1_PeriodElapsedCallback(void) { tick += UINT16_MAX; }

#ifndef QUADRATURE_ENCODER
uint32_t encoderLastWidth = 0;
void TIM2_IC_CaptureCallback(void) {
  uint32_t encoderPeriod = LL_TIM_OC_GetCompareCH1(TIM2);
  if (encoderPeriod) {
    // `if (encoderPeriod)` ensures we don't divide by 0.
    uint32_t encoderWidth = LL_TIM_OC_GetCompareCH2(TIM2);
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
#endif

static void reportCommandError(uint8_t commandID) {
  CAN_TxHeader txHeader = {.StdId = (MOCO_ID << 6) | MESSAGE_ID_ERROR,
                           .IDE = CAN_ID_STD,
                           .RTR = CAN_RTR_DATA,
                           .DLC = 1,
                           .TransmitGlobalTime = false};
  CANMessage txData = {.commandError = {commandID}};
  CAN_Tx_Mailboxes txMailbox = 0;
  CAN_AddTxMessage(&txHeader, (uint8_t *)&txData, &txMailbox);
}

static float pide(float error, float P, float I, float D, float feedForward,
                  float *lastError, float *integralError, float deltaT,
                  DebugTelemetry *debugTelemetry) {
  *integralError += error * deltaT;
  if (debugTelemetry != NULL) {
    debugTelemetry->pOut = P * error;
    debugTelemetry->iOut = I * *integralError;
    debugTelemetry->dOut = D * (error - *lastError) * deltaT;
    debugTelemetry->error = error;
    debugTelemetry->deltaT = deltaT;
  }
  float out = P * error + I * *integralError +
              D * (error - *lastError) * deltaT + feedForward;
  *lastError = error;
  return out;
}

static void ramp(float in, float *out, float rampRate, float deltaT,
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

static void lowPass(float in, float *out, float alpha, float deltaT) {
  *out += alpha * deltaT * (in - *out);
}

static int16_t clamp4(int16_t x, int16_t pMax, int16_t pMin, int16_t nMin,
                      int16_t nMax) {
  if (x < 0) {
    return x < nMax ? nMax : x > nMin ? nMin : x;
  } else {
    return x > pMax ? pMax : x < pMin ? pMin : x;
  }
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
    LL_GPIO_SetOutputPin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
    while (LL_TIM_GetCounter(TIM1) < 0x8000)
      ;
    while (LL_TIM_GetCounter(TIM1) >= 0x8000)
      ;
    while (LL_TIM_GetCounter(TIM1) < 0x8000)
      ;
    while (LL_TIM_GetCounter(TIM1) >= 0x8000)
      ;
    LL_GPIO_ResetOutputPin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
    while (LL_TIM_GetCounter(TIM1) < 0x8000)
      ;
    while (LL_TIM_GetCounter(TIM1) >= 0x8000)
      ;
    while (LL_TIM_GetCounter(TIM1) < 0x8000)
      ;
    while (LL_TIM_GetCounter(TIM1) >= 0x8000)
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
