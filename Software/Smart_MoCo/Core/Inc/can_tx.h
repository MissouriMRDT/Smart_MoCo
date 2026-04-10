#ifndef __CAN_TX_H
#define __CAN_TX_H

#include "smoco.h"

void CAN_TX_QueueCalibrated(void);
void CAN_TX_QueueCommandError(uint8_t commandID);
void CAN_TX_UpdateTelemetry(SMOCOMessage message);
void CAN_TX_SendTelemetry(void);
void CAN_TX_SendDebugTelemetry(void);
void CAN_TX_RequestMissingParameter(void);

#endif /* __CAN_TX_H */
