#include "can.h"

#include "main.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_utils.h"
#include <stdint.h>

uint32_t CAN_TIMEOUT_VALUE = 20000; // 10ms
void CAN_Init(void) {
  // Peripheral clock enable
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**CAN GPIO Configuration
      PA11     ------> CAN_RX
      PA12     ------> CAN_TX
  */
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11 | LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Request initialisation
  SET_BIT(CAN->MCR, CAN_MCR_INRQ);

  // Get tick
  uint32_t tickstart = GetTick();

  // Wait initialisation acknowledge
  while ((CAN->MSR & CAN_MSR_INAK) == 0U) {
    if ((GetTick() - tickstart) > CAN_TIMEOUT_VALUE) {
      Error_Handler();
    }
  }

  // Exit from sleep mode
  CLEAR_BIT(CAN->MCR, CAN_MCR_SLEEP);

  // Get tick
  tickstart = GetTick();

  // Check Sleep mode leave acknowledge
  while ((CAN->MSR & CAN_MSR_SLAK) != 0U) {
    if ((GetTick() - tickstart) > CAN_TIMEOUT_VALUE) {
      Error_Handler();
    }
  }

  // Set the master control register
  // Mode: Normal
  // Time Triggered Mode: Disabled
  // Auto Bus Off: Disabled
  // Auto Wake Up: Disabled
  // Auto Retransmission: Enabled
  // Receive FIFO Locked: Disabled
  // Transmit FIFO Priority: Disabled
  //
  CLEAR_BIT(CAN->MCR, CAN_MCR_TTCM | CAN_MCR_ABOM | CAN_MCR_AWUM |
                          CAN_MCR_NART | CAN_MCR_RFLM | CAN_MCR_TXFP);

  // Set the bit timing register
  // Prescaler: 4 (Time Quanta: 500ns, Baud Rate: 125000)
  // ReSynchronization Jump Width: 4;
  // Time Quanta in Bit Segment 1: 8;
  // Time Quanta in Bit Segment 2: 7;
  WRITE_REG(CAN->BTR,
            (uint32_t)(CAN_BTR_SJW |
                       (CAN_BTR_TS1_2 | CAN_BTR_TS1_1 | CAN_BTR_TS1_0) |
                       (CAN_BTR_TS2_2 | CAN_BTR_TS2_1) | 3U));
}

/**
 * @param filterBank filter bank 0-13
 * @param filterEnable true: enable filter, false: disable filter
 * @param filterFIFOAssignment filter assignment
 * @param filterModeList true: list mode, false: mask mode
 * @param filterScale32Bit true: one 32-bit filter, false: two 16-bit filters
 * @param filterId see RM0091 Figure 319
 * @param filterMask see RM0091 Figure 319
 */

void CAN_ConfigFilter(uint8_t filterBank, bool filterEnable,
                      CAN_receive_FIFO_number filterFIFOAssignment,
                      bool filterModeList, bool filterScale32Bit,
                      uint32_t filterId, uint32_t filterMask) {
  uint32_t filternbrbitpos = (uint32_t)1 << (filterBank & 0x1FU);

  // Initialisation mode for the filter
  SET_BIT(CAN->FMR, CAN_FMR_FINIT);

  // Filter Deactivation
  CLEAR_BIT(CAN->FA1R, filternbrbitpos);

  // Filter Scale
  if (filterScale32Bit) {
    // 32-bit scale for the filter
    SET_BIT(CAN->FS1R, filternbrbitpos);

    // 32-bit identifier or First 32-bit identifier
    CAN->sFilterRegister[filterBank].FR1 = filterId;

    // 32-bit mask or Second 32-bit identifier
    CAN->sFilterRegister[filterBank].FR2 = filterMask;
  } else {
    // 16-bit scale for the filter
    CLEAR_BIT(CAN->FS1R, filternbrbitpos);

    // First 16-bit identifier and First 16-bit mask
    // Or First 16-bit identifier and Second 16-bit identifier
    CAN->sFilterRegister[filterBank].FR1 =
        (filterMask << 16U) | (0x0000FFFFU & filterId);

    // Second 16-bit identifier and Second 16-bit mask
    // Or Third 16-bit identifier and Fourth 16-bit identifier
    CAN->sFilterRegister[filterBank].FR2 =
        (0xFFFF0000U & filterMask) | (filterId >> 16U);
  }

  // Filter Mode
  if (filterModeList) {
    // Identifier list mode for the filter
    SET_BIT(CAN->FM1R, filternbrbitpos);
  } else {
    // Id/Mask mode for the filter
    CLEAR_BIT(CAN->FM1R, filternbrbitpos);
  }

  // Filter FIFO assignment
  if (filterFIFOAssignment == CAN_RX_FIFO0) {
    // FIFO 0 assignation for the filter
    CLEAR_BIT(CAN->FFA1R, filternbrbitpos);
  } else {
    // FIFO 1 assignation for the filter
    SET_BIT(CAN->FFA1R, filternbrbitpos);
  }

  // Filter activation
  if (filterEnable)
    SET_BIT(CAN->FA1R, filternbrbitpos);

  // Leave the initialisation mode for the filter
  CLEAR_BIT(CAN->FMR, CAN_FMR_FINIT);
}

void CAN_Start(void) {
  // Request leave initialisation
  CLEAR_BIT(CAN->MCR, CAN_MCR_INRQ);

  uint32_t tickstart = GetTick();

  // Wait the acknowledge
  while ((CAN->MSR & CAN_MSR_INAK) != 0U) {
    // Check for the Timeout
    if ((GetTick() - tickstart) > CAN_TIMEOUT_VALUE) {
      Error_Handler();
    }
  }
}

// Return the number of free Tx Mailboxes.
uint8_t CAN_GetTxMailboxesFreeLevel(void) {
  uint8_t freelevel = 0U;
  if ((CAN->TSR & CAN_TSR_TME0) != 0U)
    freelevel++;

  if ((CAN->TSR & CAN_TSR_TME1) != 0U)
    freelevel++;

  if ((CAN->TSR & CAN_TSR_TME2) != 0U)
    freelevel++;

  return freelevel;
}

/**
 * @brief  Add a message to the first free Tx mailbox and activate the
 * corresponding transmission request.
 * @param  header pointer to a CAN_TxHeader structure.
 * @param  data array containing the payload of the Tx frame.
 * @param  txMailbox pointer to a variable where the function will return the
 * TxMailbox used to store the Tx message.
 * @retval true: message added, false: no free Tx mailbox
 */
bool CAN_AddTxMessage(const CAN_TxHeader *header, const uint8_t data[],
                      CAN_Tx_Mailboxes *txMailbox) {
  uint32_t transmitmailbox;
  uint32_t tsr = READ_REG(CAN->TSR);

  // Check that all the Tx mailboxes are not full
  if (((tsr & CAN_TSR_TME0) == 0U) && ((tsr & CAN_TSR_TME1) == 0U) &&
      ((tsr & CAN_TSR_TME2) == 0U))
    return false;

  // Select an empty transmit mailbox
  transmitmailbox = (tsr & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;

  // Store the Tx mailbox
  *txMailbox = (uint32_t)1 << transmitmailbox;

  // Set up the Id
  if (header->IDE == CAN_ID_STD) {
    CAN->sTxMailBox[transmitmailbox].TIR =
        ((header->StdId << CAN_TI0R_STID_Pos) | header->RTR);
  } else {
    CAN->sTxMailBox[transmitmailbox].TIR =
        ((header->ExtId << CAN_TI0R_EXID_Pos) | header->IDE | header->RTR);
  }

  // Set up the DLC
  CAN->sTxMailBox[transmitmailbox].TDTR = (header->DLC);

  // Set up the Transmit Global Time mode
  if (header->TransmitGlobalTime) {
    SET_BIT(CAN->sTxMailBox[transmitmailbox].TDTR, CAN_TDT0R_TGT);
  }

  // Set up the data field
  WRITE_REG(CAN->sTxMailBox[transmitmailbox].TDHR,
            ((uint32_t)data[7] << CAN_TDH0R_DATA7_Pos) |
                ((uint32_t)data[6] << CAN_TDH0R_DATA6_Pos) |
                ((uint32_t)data[5] << CAN_TDH0R_DATA5_Pos) |
                ((uint32_t)data[4] << CAN_TDH0R_DATA4_Pos));
  WRITE_REG(CAN->sTxMailBox[transmitmailbox].TDLR,
            ((uint32_t)data[3] << CAN_TDL0R_DATA3_Pos) |
                ((uint32_t)data[2] << CAN_TDL0R_DATA2_Pos) |
                ((uint32_t)data[1] << CAN_TDL0R_DATA1_Pos) |
                ((uint32_t)data[0] << CAN_TDL0R_DATA0_Pos));

  // Request transmission
  SET_BIT(CAN->sTxMailBox[transmitmailbox].TIR, CAN_TI0R_TXRQ);

  // Return function status
  return true;
}

// Return Rx FIFO fill level.
uint32_t CAN_GetRxFifoFillLevel(CAN_receive_FIFO_number rxFIFO) {
  if (rxFIFO == CAN_RX_FIFO0)
    return CAN->RF0R & CAN_RF0R_FMP0;
  return CAN->RF1R & CAN_RF1R_FMP1;
}

/**
 * @brief  Get an CAN frame from the Rx FIFO zone into the message RAM.
 * @param  rxFIFO
 * @param  header pointer to a CAN_RxHeader structure where the header of the Rx
 * frame will be stored.
 * @param  data array where the payload of the Rx frame will be stored.
 * @retval true: message received, false: FIFO empty
 */
bool CAN_GetRxMessage(CAN_receive_FIFO_number rxFIFO, CAN_RxHeader *header,
                      uint8_t data[]) {
  // Check the Rx FIFO
  if (rxFIFO == CAN_RX_FIFO0) {
    // Check that the Rx FIFO 0 is not empty
    if ((CAN->RF0R & CAN_RF0R_FMP0) == 0U)
      return false;
  } else {
    // Check that the Rx FIFO 1 is not empty
    if ((CAN->RF1R & CAN_RF1R_FMP1) == 0U)
      return false;
  }

  // Get the header
  header->IDE = CAN_RI0R_IDE & CAN->sFIFOMailBox[rxFIFO].RIR;
  if (header->IDE == CAN_ID_STD) {
    header->StdId =
        (CAN_RI0R_STID & CAN->sFIFOMailBox[rxFIFO].RIR) >> CAN_TI0R_STID_Pos;
  } else {
    header->ExtId =
        ((CAN_RI0R_EXID | CAN_RI0R_STID) & CAN->sFIFOMailBox[rxFIFO].RIR) >>
        CAN_RI0R_EXID_Pos;
  }
  header->RTR = (CAN_RI0R_RTR & CAN->sFIFOMailBox[rxFIFO].RIR);
  if (((CAN_RDT0R_DLC & CAN->sFIFOMailBox[rxFIFO].RDTR) >> CAN_RDT0R_DLC_Pos) >=
      8U) {
    // Truncate DLC to 8 if received field is over range
    header->DLC = 8U;
  } else {
    header->DLC =
        (CAN_RDT0R_DLC & CAN->sFIFOMailBox[rxFIFO].RDTR) >> CAN_RDT0R_DLC_Pos;
  }
  header->FilterMatchIndex =
      (CAN_RDT0R_FMI & CAN->sFIFOMailBox[rxFIFO].RDTR) >> CAN_RDT0R_FMI_Pos;
  header->Timestamp =
      (CAN_RDT0R_TIME & CAN->sFIFOMailBox[rxFIFO].RDTR) >> CAN_RDT0R_TIME_Pos;

  /* Get the data */
  data[0] = (uint8_t)((CAN_RDL0R_DATA0 & CAN->sFIFOMailBox[rxFIFO].RDLR) >>
                      CAN_RDL0R_DATA0_Pos);
  data[1] = (uint8_t)((CAN_RDL0R_DATA1 & CAN->sFIFOMailBox[rxFIFO].RDLR) >>
                      CAN_RDL0R_DATA1_Pos);
  data[2] = (uint8_t)((CAN_RDL0R_DATA2 & CAN->sFIFOMailBox[rxFIFO].RDLR) >>
                      CAN_RDL0R_DATA2_Pos);
  data[3] = (uint8_t)((CAN_RDL0R_DATA3 & CAN->sFIFOMailBox[rxFIFO].RDLR) >>
                      CAN_RDL0R_DATA3_Pos);
  data[4] = (uint8_t)((CAN_RDH0R_DATA4 & CAN->sFIFOMailBox[rxFIFO].RDHR) >>
                      CAN_RDH0R_DATA4_Pos);
  data[5] = (uint8_t)((CAN_RDH0R_DATA5 & CAN->sFIFOMailBox[rxFIFO].RDHR) >>
                      CAN_RDH0R_DATA5_Pos);
  data[6] = (uint8_t)((CAN_RDH0R_DATA6 & CAN->sFIFOMailBox[rxFIFO].RDHR) >>
                      CAN_RDH0R_DATA6_Pos);
  data[7] = (uint8_t)((CAN_RDH0R_DATA7 & CAN->sFIFOMailBox[rxFIFO].RDHR) >>
                      CAN_RDH0R_DATA7_Pos);

  // Release the FIFO
  if (rxFIFO == CAN_RX_FIFO0)
    SET_BIT(CAN->RF0R, CAN_RF0R_RFOM0);
  else
    SET_BIT(CAN->RF1R, CAN_RF1R_RFOM1);

  return true;
}