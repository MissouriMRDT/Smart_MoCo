#ifndef __CAN_RX_H
#define __CAN_RX_H

#include "controller.h"
#include "smoco.h"
#include <stdint.h>

static const uint64_t REQUIRED_PARAMETERS[CONTROL_MODE_COUNT] = {
    [CONTROL_MODE_STOP] = 0,
    [CONTROL_MODE_OPEN_LOOP] =
        (1ULL << SMOCO_MID_RAMP_RATE) | (1ULL << SMOCO_MID_IGNORE_LIMIT) |
        (1ULL << SMOCO_MID_SOFT_LIMIT) | (1ULL << SMOCO_MID_DUTY_CYCLE_RANGE),
    [CONTROL_MODE_POSITION] = (1ULL << SMOCO_MID_PI) | (1ULL << SMOCO_MID_D) |
                              (1ULL << SMOCO_MID_IGNORE_LIMIT) |
                              (1ULL << SMOCO_MID_SOFT_LIMIT) |
                              (1ULL << SMOCO_MID_DUTY_CYCLE_RANGE),
    [CONTROL_MODE_VELOCITY] = (1ULL << SMOCO_MID_PI) | (1ULL << SMOCO_MID_D) |
                              (1ULL << SMOCO_MID_IGNORE_LIMIT) |
                              (1ULL << SMOCO_MID_SOFT_LIMIT) |
                              (1ULL << SMOCO_MID_DUTY_CYCLE_RANGE),
    [CONTROL_MODE_CURRENT] = (1ULL << SMOCO_MID_PI) | (1ULL << SMOCO_MID_D) |
                             (1ULL << SMOCO_MID_IGNORE_LIMIT) |
                             (1ULL << SMOCO_MID_SOFT_LIMIT) |
                             (1ULL << SMOCO_MID_DUTY_CYCLE_RANGE),
    [CONTROL_MODE_CALIBRATING] =
        (1ULL << SMOCO_MID_RAMP_RATE) | (1ULL << SMOCO_MID_DUTY_CYCLE_RANGE),
};

// These arrays are duplicated as the functions that access them may preempt
// CAN_FMP0_IRQHandler. CAN_FMP0_IRQHandler will write to x[!read_buffer]
// while preempting functions will read from x[read_buffer].
// CAN_FMP0_IRQHandler shall not preempt functions reading from x[read_buffer].
extern bool readBuffer;
extern uint64_t missingParameters[2];
extern ControlMode controlMode[2];
extern SMOCOMessage acceptedCommands[2][1 << SMOCO_WIDTH_MID];

#define ACCEPTED_COMMAND(x) (acceptedCommands[readBuffer][x].x##_)

void CAN_RX_Init(void);
void CAN_FMP0_IRQHandler(void);

#endif /* __CAN_RX_H */
