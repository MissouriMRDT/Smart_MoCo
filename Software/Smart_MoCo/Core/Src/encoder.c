#include "encoder.h"

#include "main.h"
#include "stm32f0xx_ll_tim.h"
#include <stdint.h>

int32_t encoderOffset = 0; // step

#ifdef QUADRATURE_ENCODER
void Encoder_Init(void) {
  LL_TIM_EnableCounter(TIM_ENCODER);
  LL_TIM_CC_EnableChannel(TIM_ENCODER, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
  LL_TIM_SetCounter(TIM_ENCODER, UINT16_MAX);
  encoderOffset = -UINT16_MAX;
}

inline int32_t Encoder_GetPosition(void) {
  return LL_TIM_GetCounter(TIM_ENCODER) + encoderOffset;
}

void Encoder_SetPosition(int32_t new) {
  encoderOffset = new - LL_TIM_GetCounter(TIM_ENCODER);
}

#else

volatile int32_t absPosition = 0; // step
volatile uint32_t absLastDutyCycle = 0;
volatile int32_t absRotations = 0; // ABSOLUTE_ENCODER_RESOLUTION * step

void TIM2_IC_CaptureCallback(void) {
  uint32_t encoderPeriod = LL_TIM_OC_GetCompareCH1(TIM_ENCODER);
  uint32_t encoderWidth = LL_TIM_OC_GetCompareCH2(TIM_ENCODER);

  if (encoderPeriod) {
    // `if (encoderPeriod)` ensures we don't divide by 0.
    uint32_t dutyCycle =
        ABSOLUTE_ENCODER_RESOLUTION * encoderWidth / encoderPeriod;

    if (absLastDutyCycle) {
      // Don't rollover on startup.
      if (absLastDutyCycle > ABSOLUTE_ENCODER_RESOLUTION * 0.8 &&
          dutyCycle < ABSOLUTE_ENCODER_RESOLUTION * 0.2) {
        // High to low rollover:
        absRotations++;
      } else if (absLastDutyCycle < ABSOLUTE_ENCODER_RESOLUTION * 0.2 &&
                 dutyCycle > ABSOLUTE_ENCODER_RESOLUTION * 0.8) {
        // Low to high rollover:
        absRotations--;
      }
    }
#ifdef ABSOLUTE_ENCODER_STARTUP_THRESHOLD
    else {
      // On startup, if the encoder position is past
      // ABSOLUTE_ENCODER_STARTUP_THRESHOLD, move the position back one
      // revolution.
      if (dutyCycle > ABSOLUTE_ENCODER_STARTUP_THRESHOLD)
        absRotations = -1;
    }
#endif

    absLastDutyCycle = dutyCycle;
    absPosition =
        ABSOLUTE_ENCODER_RESOLUTION * absRotations + dutyCycle + encoderOffset;
  }
}

void Encoder_Init(void) {
  LL_TIM_EnableIT_CC1(TIM_ENCODER);
  LL_TIM_CC_EnableChannel(TIM_ENCODER, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM_ENCODER, LL_TIM_CHANNEL_CH2);
  LL_TIM_EnableCounter(TIM_ENCODER);
}

inline int32_t Encoder_GetPosition(void) { return absPosition + encoderOffset; }

void Encoder_SetPosition(int32_t new) { encoderOffset = new - absPosition; }
#endif