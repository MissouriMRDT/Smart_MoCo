#include "can_tx.h"

#include "can.h"
#include "can_rx.h"
#include "controller.h"
#include "main.h"
#include "smoco.h"
#include <stdbool.h>
#include <stdint.h>

bool send_data(uint32_t mid, SMOCOMessage data) {
  CAN_TxHeader header = {.StdId = (SMOCO_ID << SMOCO_WIDTH_MID) | mid,
                         .IDE = CAN_ID_STD,
                         .RTR = CAN_RTR_DATA,
                         .DLC = SMOCO_WIDTH[mid],
                         .TransmitGlobalTime = false};
  CAN_Tx_Mailboxes mailbox;
  return CAN_AddTxMessage(&header, (uint8_t *)&data, &mailbox);
}

bool send_remote(uint32_t mid) {
  CAN_TxHeader header = {.StdId = (SMOCO_ID << SMOCO_WIDTH_MID) | mid,
                         .IDE = CAN_ID_STD,
                         .RTR = CAN_RTR_REMOTE,
                         .DLC = 0,
                         .TransmitGlobalTime = false};
  uint8_t data[8] = {0};
  CAN_Tx_Mailboxes txMailbox;
  return CAN_AddTxMessage(&header, data, &txMailbox);
}

bool queuedCalibrated = false;
;
void CAN_TX_QueueCalibrated(void) { queuedCalibrated = true; }

bool queuedCommandError = false;
uint8_t commandErrorID = 0;
void CAN_TX_QueueCommandError(uint8_t commandID) {
  queuedCommandError = true;
  commandErrorID = commandID;
}

SMOCOMessage queuedTelemetry;
void CAN_TX_UpdateTelemetry(SMOCOMessage message) { queuedTelemetry = message; }

void CAN_TX_SendTelemetry(void) {
  if (queuedCalibrated &&
      send_data(SMOCO_MID_POSITION_CALIBRATED, (SMOCOMessage){0}))
    queuedCalibrated = false;

  if (queuedCommandError &&
      send_data(
          commandErrorID,
          (SMOCOMessage){.SMOCO_MID_ERROR_ = {.commandID = commandErrorID}}))
    queuedCommandError = false;

  send_data(SMOCO_MID_POSITION, queuedTelemetry);
}

void CAN_TX_SendDebugTelemetry(void) {
  return; // TODO: enable
  if (!ACCEPTED_COMMAND(SMOCO_MID_DEBUG).enable)
    return;

  DebugTelemetry captured = debugTelemetry;

  uint64_t timeout = GetTick() + 40 * TICKS_PER_MS;
  for (uint8_t i = 0; i < 9; i++) {
    CAN_TxHeader header = {.StdId = SMOCO_ID_DEBUG | i,
                           .IDE = CAN_ID_STD,
                           .RTR = CAN_RTR_DATA,
                           .DLC = SMOCO_WIDTH_DEBUG,
                           .TransmitGlobalTime = false};

    CAN_Tx_Mailboxes mailbox;
    while (!CAN_AddTxMessage(&header, (uint8_t *)&captured + i * 4, &mailbox)) {
      if (GetTick() >= timeout)
        return;
    }
  }
}

uint8_t nextMissing = 0;
void CAN_TX_RequestMissingParameter(void) {
  uint64_t missing = missingParameters[readBuffer];
  if (missing == 0)
    return;

  if ((missing & (1ULL << nextMissing)) == 0) {
    // nextMissingParameter isn't missing. Find the next one
    nextMissing = (nextMissing + 1) % 64;
    // WARNING: this loop will be infinite if there are no set bits in
    // missing. The early return above should prevent this.
    while (!(missing & (1ULL << nextMissing)))
      nextMissing = (nextMissing + 1) % 64;
  }

  send_remote(nextMissing);

  // Cycle through the missing parameters.
  nextMissing = (nextMissing + 1) % 64;
}
