#include "can.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	INIT_PACKET_ORG();//this is needed to write the data from the packets.

	uint8_t data[8];
	CAN_RxHeaderTypeDef header;

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
		return;

	if (header.StdId < CAN_ID || header.StdId > CAN_ID + PARAM_SIZE)
		return;

	memcpy(CAN_PACKETS[header.StdId - CAN_ID], data, CAN_PACKETS_SIZE[header.StdId - CAN_ID]);
}

