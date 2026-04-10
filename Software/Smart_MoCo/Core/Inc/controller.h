#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "smoco.h"

typedef enum ControlMode {
  CONTROL_MODE_STOP,
  CONTROL_MODE_OPEN_LOOP,
  CONTROL_MODE_POSITION,
  CONTROL_MODE_VELOCITY,
  CONTROL_MODE_CURRENT,
  CONTROL_MODE_CALIBRATING,
  CONTROL_MODE_COUNT, // Sentinel
} ControlMode;

extern DebugTelemetry debugTelemetry;

void TIM17_PeriodElapsedCallback(void);
void Controller_Init(void);
void Controller_SetStatusLED(uint32_t timeout);
void Controller_ResetPID(void);

#endif /* __CONTROLLER_H */
