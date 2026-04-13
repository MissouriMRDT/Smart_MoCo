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

volatile uint32_t encoderPeriod;
volatile uint32_t encoderWidth;

void TIM2_IC_CaptureCallback(void) {
  encoderPeriod = LL_TIM_OC_GetCompareCH1(TIM_ENCODER);
  encoderWidth = LL_TIM_OC_GetCompareCH2(TIM_ENCODER);
}

volatile int32_t absPosition = 0; // step
volatile uint32_t absLastWidth = 0;
volatile int32_t absRotations = 0; // ABSOLUTE_ENCODER_RESOLUTION * step
// Should be called often enough to not miss rollover. Updates the position
// returned by Encoder_GetPosition
void Encoder_UpdatePosition(void) {
  if (encoderPeriod) {
    // `if (encoderPeriod)` ensures we don't divide by 0.
    if (absLastWidth) {
      // Don't rollover on startup.
      if (absLastWidth > encoderPeriod * 0.8 &&
          encoderWidth < encoderPeriod * 0.2) {
        // High to low rollover:
        absRotations++;
      } else if (absLastWidth < encoderPeriod * 0.2 &&
                 encoderWidth > encoderPeriod * 0.8) {
        // Low to high rollover:
        absRotations--;
      }
    }
#ifdef ABSOLUTE_ENCODER_STARTUP_THRESHOLD
    else {
      // On startup, if the encoder position is past
      // ABSOLUTE_ENCODER_STARTUP_THRESHOLD, move the position back one
      // revolution.
      if (ABSOLUTE_ENCODER_RESOLUTION * encoderWidth / encoderPeriod >
          ABSOLUTE_ENCODER_STARTUP_THRESHOLD)
        absRotations = -1;
    }
#endif
    absLastWidth = encoderWidth;
    absPosition = ABSOLUTE_ENCODER_RESOLUTION * absRotations +
                  ABSOLUTE_ENCODER_RESOLUTION * encoderWidth / encoderPeriod +
                  encoderOffset;
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