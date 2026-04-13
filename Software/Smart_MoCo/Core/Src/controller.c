#include "controller.h"

#include "can.h"
#include "can_rx.h"
#include "can_tx.h"
#include "encoder.h"
#include "main.h"
#include "smoco.h"
#include "stm32f0xx_ll_tim.h"
#include <stdint.h>

static float pide(float error, float P, float I, float D, float feedForward,
                  float *lastError, float *integralError, float deltaT) {
  *integralError += error * deltaT;
  if (ACCEPTED_COMMAND(SMOCO_MID_DEBUG).enable) {
    debugTelemetry.pOut = P * error;
    debugTelemetry.iOut = I * *integralError;
    debugTelemetry.dOut = D * (error - *lastError) * deltaT;
    debugTelemetry.error = error;
    debugTelemetry.deltaT = deltaT;
  }
  float out = P * error + I * *integralError +
              D * (error - *lastError) * deltaT + feedForward;
  *lastError = error;
  return out;
}

static void ramp(float in, float *out, float rampRate, float deltaT) {
  if (ACCEPTED_COMMAND(SMOCO_MID_DEBUG).enable) {
    debugTelemetry.pOut = *out;
    debugTelemetry.iOut = 0;
    debugTelemetry.dOut = 0;
    debugTelemetry.error = in - *out;
    debugTelemetry.deltaT = deltaT;
  }
  if (in > *out + rampRate * deltaT) {
    *out += rampRate * deltaT;
  } else if (in < *out - rampRate * deltaT) {
    *out -= rampRate * deltaT;
  } else {
    *out = in;
  }
}

static int16_t clamp4(int16_t x, int16_t pMax, int16_t pMin, int16_t nMin,
                      int16_t nMax) {
  if (x < 0) {
    return x < nMax ? nMax : x > nMin ? 0 : x;
  } else {
    return x > pMax ? pMax : x < pMin ? 0 : x;
  }
}

uint64_t statusOffTime = UINT64_MAX;
float pidLastError = 0;
float pidIntegralError = 0;
float pwm = 0;
DebugTelemetry debugTelemetry = {0};

void TIM17_PeriodElapsedCallback(void) {
  uint64_t now = GetTick();
  float deltaT = 0.005; // Function is called every 5ms by a timer

  // Update status LED
  if (statusOffTime < now) {
    statusOffTime = UINT32_MAX;
    LL_GPIO_ResetOutputPin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
  }

// Update controller
#ifndef QUADRATURE_ENCODER
  Encoder_UpdatePosition();
#endif

  int32_t position = Encoder_GetPosition();
  // TODO: update
  float velocity = 0;
  uint8_t current = LL_ADC_REG_ReadConversionData32(ADC1) & 0x00FF;

  // Read limit switches
  bool limitA = (LL_GPIO_ReadInputPort(LIM_A_GPIO_Port) & LIM_A_Pin) != 0;
  bool limitB = (LL_GPIO_ReadInputPort(LIM_B_GPIO_Port) & LIM_B_Pin) != 0;

  bool softLimitA =
      position <= ACCEPTED_COMMAND(SMOCO_MID_SOFT_LIMIT).aPosition;
  bool softLimitB =
      position >= ACCEPTED_COMMAND(SMOCO_MID_SOFT_LIMIT).bPosition;

  if (controlMode[readBuffer] == CONTROL_MODE_OPEN_LOOP) {
    pwm = pwm < -1 ? -1 : pwm > 1 ? 1 : pwm;
    ramp((float)ACCEPTED_COMMAND(SMOCO_MID_OPEN_LOOP).dutyCycle / 32768, &pwm,
         ACCEPTED_COMMAND(SMOCO_MID_RAMP_RATE).rampRate, deltaT);
  } else if (controlMode[readBuffer] == CONTROL_MODE_CALIBRATING) {
    pwm = pwm < -1 ? -1 : pwm > 1 ? 1 : pwm;
    ramp((float)ACCEPTED_COMMAND(SMOCO_MID_CALIBRATE).dutyCycle / 32768, &pwm,
         ACCEPTED_COMMAND(SMOCO_MID_RAMP_RATE).rampRate, deltaT);
    if (pwm < 0 ? limitA : limitB) {
      Encoder_SetPosition(
          ACCEPTED_COMMAND(SMOCO_MID_CALIBRATE).limitSwitchPosition);
      controlMode[readBuffer] = CONTROL_MODE_STOP;
      controlMode[!readBuffer] = CONTROL_MODE_STOP;
      CAN_TX_QueueCalibrated();
    }
  } else {
    float error = 0;
    float feedForward = 0;
    switch (controlMode[readBuffer]) {
    case CONTROL_MODE_POSITION:
      error = ACCEPTED_COMMAND(SMOCO_MID_TARGET_POSITION).position - position;
      feedForward = ACCEPTED_COMMAND(SMOCO_MID_TARGET_POSITION).feedForward;
      break;
    case CONTROL_MODE_VELOCITY:
      error =
          0; // ACCEPTED_COMMAND(SMOCO_MID_TARGET_VELOCITY).velocity - velocity;
      feedForward = ACCEPTED_COMMAND(SMOCO_MID_TARGET_VELOCITY).feedForward;
      break;
    case CONTROL_MODE_CURRENT:
      error = 0; // ACCEPTED_COMMAND(SMOCO_MID_TARGET_CURRENT).current -
                 // current;
      feedForward = ACCEPTED_COMMAND(SMOCO_MID_TARGET_CURRENT).feedForward;
    default:
      break;
    }
    pwm =
        pide(error, ACCEPTED_COMMAND(SMOCO_MID_PI).p,
             ACCEPTED_COMMAND(SMOCO_MID_PI).i, ACCEPTED_COMMAND(SMOCO_MID_D).d,
             feedForward, &pidLastError, &pidIntegralError, deltaT);
  }

  bool motorA = false;
  bool motorB = false;
  uint32_t ccr1 = 0;
  if (missingParameters[readBuffer] == 0 &&
      controlMode[readBuffer] != CONTROL_MODE_STOP) {
    if (pwm < -0.99999)
      pwm = -0.99999;
    if (pwm > 0.99999)
      pwm = 0.99999;
    int16_t clampedPWM = clamp4(
        pwm * INT16_MAX, ACCEPTED_COMMAND(SMOCO_MID_DUTY_CYCLE_RANGE).fwdMax,
        ACCEPTED_COMMAND(SMOCO_MID_DUTY_CYCLE_RANGE).fwdMin,
        ACCEPTED_COMMAND(SMOCO_MID_DUTY_CYCLE_RANGE).revMin,
        ACCEPTED_COMMAND(SMOCO_MID_DUTY_CYCLE_RANGE).revMax);
    if (clampedPWM < 0 && !softLimitA &&
        (ACCEPTED_COMMAND(SMOCO_MID_IGNORE_LIMIT).ab & 0b1 || !limitA)) {
      motorA = false;
      motorB = true;
      ccr1 = -clampedPWM * 2;
    } else if (clampedPWM > 0 && !softLimitB &&
               (ACCEPTED_COMMAND(SMOCO_MID_IGNORE_LIMIT).ab & 0b10 ||
                !limitB)) {
      motorA = true;
      motorB = false;
      ccr1 = clampedPWM * 2;
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
  LL_TIM_OC_SetCompareCH1(TIM_MOTOR, ccr1);

  // TODO: Current (A/8 u8)
  CAN_TX_UpdateTelemetry((SMOCOMessage){
      .SMOCO_MID_POSITION_ = {.position = position,
                              .velocity = velocity,
                              .current = current,
                              .flags = limitA | (limitB << 1) |
                                       (softLimitA << 2) | (softLimitB << 3)}});

  if (ACCEPTED_COMMAND(SMOCO_MID_DEBUG).enable) {
    debugTelemetry.tick = now;
    debugTelemetry.position = position;
    debugTelemetry.velocity = velocity;
    debugTelemetry.current = current;
  }
}

void Controller_Init(void) {
  LL_TIM_EnableIT_UPDATE(TIM_OUTPUT);
  LL_TIM_EnableCounter(TIM_OUTPUT);

  LL_TIM_CC_EnableChannel(TIM_MOTOR, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableCounter(TIM_MOTOR);
}

void Controller_SetStatusLED(uint64_t timeout) {
  statusOffTime = GetTick() + timeout;
  LL_GPIO_SetOutputPin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
}

void Controller_ResetPID(void) {
  pidLastError = 0;
  pidIntegralError = 0;
}
