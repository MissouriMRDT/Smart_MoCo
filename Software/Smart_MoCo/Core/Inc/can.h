#ifndef __CAN_H__
#define __CAN_H__

#include <stdbool.h>
#include <stdint.h>

// CAN Identifier Type
typedef enum {
  CAN_ID_STD = 0U, // Standard Id
  CAN_ID_EXT = 1U, // Extended Id
} CAN_identifier_type;

// CAN Remote Transmission Request
typedef enum {
  CAN_RTR_DATA = 0x00000000U,   // Data frame
  CAN_RTR_REMOTE = 0x00000002U, // Remote frame
} CAN_remote_transmission_request;

// CAN Receive FIFO Number
typedef enum {
  CAN_RX_FIFO0 = 0x00000000U, // CAN receive FIFO 0
  CAN_RX_FIFO1 = 0x00000001U, // CAN receive FIFO 1
} CAN_receive_FIFO_number;

// CAN Tx Mailboxes
typedef enum {
  CAN_TX_MAILBOX0 = 0x00000001U, // Tx Mailbox 0
  CAN_TX_MAILBOX1 = 0x00000002U, // Tx Mailbox 1
  CAN_TX_MAILBOX2 = 0x00000004U, // Tx Mailbox 2
} CAN_Tx_Mailboxes;

typedef struct {
  uint32_t StdId;                      // Standard identifier 0-0x7FF.
  uint32_t ExtId;                      // Extended identifier 0-0x1FFFFFFF.
  CAN_identifier_type IDE;             // 0: Standard ID, 1: Extended ID
  CAN_remote_transmission_request RTR; // 0: Data, 1: Remote
  uint32_t DLC;                        // Data length 0-8
  uint32_t
      Timestamp; // Timestamp counter value 0-0xFFFF captured on start of frame
                 // reception if Time Triggered Communication Mode is enabled.
  uint32_t
      FilterMatchIndex; // Index of matching acceptance filter element 0-0xFF.

} CAN_RxHeader;

typedef struct {
  uint32_t StdId;                      // Standard identifier 0-0x7FF.
  uint32_t ExtId;                      // Extended identifier 0-0x1FFFFFFF.
  CAN_identifier_type IDE;             // 0: Standard ID, 1: Extended ID
  CAN_remote_transmission_request RTR; // 0: Data, 1: Remote
  uint32_t DLC;                        // Data length 0-8
  bool TransmitGlobalTime; // Transmit timestamp counter value in DATA6
                           // and DATA7 if Time Triggered Communication Mode
                           // is enabled and DLC == 8.
} CAN_TxHeader;

void CAN_Init(void);
void CAN_ConfigFilter(uint8_t filterBank, bool filterEnable,
                      CAN_receive_FIFO_number filterFIFOAssignment,
                      bool filterModeList, bool filterScale32Bit,
                      uint32_t filterId, uint32_t filterMask);
void CAN_Start(void);
uint8_t CAN_GetTxMailboxesFreeLevel(void);
bool CAN_AddTxMessage(const CAN_TxHeader *header, const uint8_t data[],
                      CAN_Tx_Mailboxes *txMailbox);
uint32_t CAN_GetRxFifoFillLevel(CAN_receive_FIFO_number rxFIFO);
bool CAN_GetRxMessage(CAN_receive_FIFO_number rxFIFO, CAN_RxHeader *header,
                      uint8_t data[]);

#endif /*__ __CAN_H__ */
