#include "can_rx.h"

#include "can.h"
#include "can_tx.h"
#include "controller.h"
#include "main.h"
#include "smoco.h"
#include "stm32f042x6.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Bitset of parameters that have been set
static_assert(SMOCO_WIDTH_MID <= 6,
              "setParameters implementation depends on SMOCO_WIDTH_MID <= 6");
uint64_t setParameters = 0;

bool readBuffer = false;
uint64_t missingParameters[2] = {0, 0};
ControlMode controlMode[2] = {CONTROL_MODE_STOP, CONTROL_MODE_STOP};
SMOCOMessage acceptedCommands[2][1 << SMOCO_WIDTH_MID];

bool lastPIDUsed = false;

void CAN_RX_Init(void) {
  CAN_Init();
  // filterId = STID << 21
  // maskId = STID << 21
  // Filter messages with STID[10:10-SMOCO_WIDTH_DID] == SMOCO_ID
  CAN_ConfigFilter(0, true, CAN_RX_FIFO0, false, true,
                   (SMOCO_ID << SMOCO_WIDTH_MID) << 21,
                   (((1 << SMOCO_WIDTH_DID) - 1) << SMOCO_WIDTH_MID) << 21);
  NVIC_SetPriority(CEC_CAN_IRQn, 2);
  NVIC_EnableIRQ(CEC_CAN_IRQn);
  SET_BIT(CAN->IER, CAN_IER_FMPIE0);
  CAN_Start();
}

// Not reentrant. May be preempted. Shall not preempt any function reading from
// x[readBuffer]
void CAN_FMP0_IRQHandler(void) {
  CAN_RxHeader header;
  SMOCOMessage data;

  // Receive a single message from the top of FIFO0
  if (!CAN_GetRxMessage(CAN_RX_FIFO0, &header, (uint8_t *)&data))
    return;

  // Message must be a standard data frame.
  if (header.RTR != CAN_RTR_DATA || header.IDE != CAN_ID_STD)
    return;

  uint32_t messageID = header.StdId & ((1 << SMOCO_WIDTH_MID) - 1);

  // Check message length
  if (header.DLC < SMOCO_WIDTH[messageID]) {
    goto error_handler;
  }

  switch (messageID) {
  case SMOCO_MID_STOP:
    controlMode[!readBuffer] = CONTROL_MODE_STOP;
    missingParameters[!readBuffer] = REQUIRED_PARAMETERS[CONTROL_MODE_STOP];
    acceptedCommands[!readBuffer][SMOCO_MID_PI].SMOCO_MID_PI_.p = 0;
    acceptedCommands[!readBuffer][SMOCO_MID_PI].SMOCO_MID_PI_.i = 0;
    acceptedCommands[!readBuffer][SMOCO_MID_D].SMOCO_MID_D_.d = 0;
    acceptedCommands[!readBuffer][SMOCO_MID_RAMP_RATE]
        .SMOCO_MID_RAMP_RATE_.rampRate = 10;
    lastPIDUsed = false;
    setParameters = 0;
    Controller_ResetPID();
    break;
  case SMOCO_MID_PI:
  case SMOCO_MID_D:
    // Set PID logic
    // +-----+      +---+         +----+
    // |Unset|-PID->|Set|-Target->|Used|
    // +-----+      +---+         +----+
    //   ^ ^--Reset--/ ^----PID----/ |
    //   \---Reset OR Other Target---/
    if (lastPIDUsed)
      lastPIDUsed = false;
    break;
  case SMOCO_MID_SOFT_LIMIT:
    if (data.SMOCO_MID_SOFT_LIMIT_.aPosition >=
        data.SMOCO_MID_SOFT_LIMIT_.bPosition) {
      // Limit A must be less than limit B
      goto error_handler;
    }
    break;
  case SMOCO_MID_CALIBRATE:
    controlMode[!readBuffer] = CONTROL_MODE_CALIBRATING;
    missingParameters[!readBuffer] =
        REQUIRED_PARAMETERS[CONTROL_MODE_CALIBRATING];
    break;
  case SMOCO_MID_DUTY_CYCLE_RANGE:
    if (data.SMOCO_MID_DUTY_CYCLE_RANGE_.fwdMax <=
            data.SMOCO_MID_DUTY_CYCLE_RANGE_.fwdMin ||
        data.SMOCO_MID_DUTY_CYCLE_RANGE_.fwdMin <
            data.SMOCO_MID_DUTY_CYCLE_RANGE_.revMin ||
        data.SMOCO_MID_DUTY_CYCLE_RANGE_.revMin <=
            data.SMOCO_MID_DUTY_CYCLE_RANGE_.revMax) {
      goto error_handler;
    }
    break;
  case SMOCO_MID_ECHO_REQUEST: {
    Controller_SetStatusLED(200 * TICKS_PER_MS, 0xffff, 0xffff, 0x0000);
    CAN_TxHeader txHeader = {.StdId = (SMOCO_ID << 6) | SMOCO_MID_ECHO_REPLY,
                             .IDE = CAN_ID_STD,
                             .RTR = CAN_RTR_DATA,
                             .DLC = header.DLC,
                             .TransmitGlobalTime = DISABLE};
    CAN_Tx_Mailboxes txMailbox;
    CAN_AddTxMessage(&txHeader,
                     (uint8_t *)&data.SMOCO_MID_ECHO_REQUEST_.payload,
                     &txMailbox);
    break;
  }
  case SMOCO_MID_OPEN_LOOP:
    controlMode[!readBuffer] = CONTROL_MODE_OPEN_LOOP;
    missingParameters[!readBuffer] =
        REQUIRED_PARAMETERS[CONTROL_MODE_OPEN_LOOP];
    break;
  case SMOCO_MID_TARGET_POSITION:
    if (controlMode[!readBuffer] != CONTROL_MODE_POSITION) {
      Controller_ResetPID();
      if (lastPIDUsed)
        setParameters &= ~((1 << SMOCO_MID_PI) | (1 << SMOCO_MID_D));
    }
    lastPIDUsed = true;
    controlMode[!readBuffer] = CONTROL_MODE_POSITION;
    missingParameters[!readBuffer] = REQUIRED_PARAMETERS[CONTROL_MODE_POSITION];
    break;
  case SMOCO_MID_TARGET_VELOCITY:
    if (controlMode[!readBuffer] != CONTROL_MODE_VELOCITY) {
      Controller_ResetPID();
      if (lastPIDUsed)
        setParameters &= ~((1 << SMOCO_MID_PI) | (1 << SMOCO_MID_D));
    }
    lastPIDUsed = true;
    controlMode[!readBuffer] = CONTROL_MODE_VELOCITY;
    missingParameters[!readBuffer] = REQUIRED_PARAMETERS[CONTROL_MODE_VELOCITY];
    break;
  case SMOCO_MID_TARGET_CURRENT:
    if (controlMode[!readBuffer] != CONTROL_MODE_CURRENT) {
      Controller_ResetPID();
      if (lastPIDUsed)
        setParameters &= ~((1 << SMOCO_MID_PI) | (1 << SMOCO_MID_D));
    }
    lastPIDUsed = true;
    controlMode[!readBuffer] = CONTROL_MODE_CURRENT;
    missingParameters[!readBuffer] = REQUIRED_PARAMETERS[CONTROL_MODE_CURRENT];
    break;
  default:
    break;
  }

  // Record this message as accepted
  acceptedCommands[!readBuffer][messageID] = data;
  // Parameter messageID is set
  setParameters |= 1 << messageID;
  // Parameters are missing only if they were before and aren't set
  missingParameters[!readBuffer] &= ~setParameters;

  // Swap read and write buffers
  readBuffer = !readBuffer;
  // Copy read buffer to write buffer
  missingParameters[!readBuffer] = missingParameters[readBuffer];
  controlMode[!readBuffer] = controlMode[readBuffer];
  memcpy(acceptedCommands[!readBuffer], acceptedCommands[readBuffer],
         sizeof(acceptedCommands[0]));

  if (messageID == SMOCO_MID_ECHO_REQUEST)
    Controller_SetStatusLED(200 * TICKS_PER_MS, 0xffff, 0xffff, 0x0000);
  else
    Controller_SetStatusLED(100 * TICKS_PER_MS, 0x0000, 0xffff, 0xffff);

  return;

error_handler:
  Controller_SetStatusLED(UINT64_MAX, 0xffff, 0x0000, 0x0000);
  CAN_TX_QueueCommandError(messageID);
}
