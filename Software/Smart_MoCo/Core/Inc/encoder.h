#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

#ifndef QUADRATURE_ENCODER
void TIM2_IC_CaptureCallback(void);
#endif

void Encoder_Init(void);
int32_t Encoder_GetPosition(void);
void Encoder_SetPosition(int32_t new);

#endif /* __ENCODER_H */
