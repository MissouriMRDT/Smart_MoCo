#include "can.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint64_t data;
	CAN_RxHeaderTypeDef header;

	if (HAL_CAN_getRxMessage(hcan, CAN_RX_FIFO0, &header, &data) != HAL_OK)
		return;

	if (header.StdId < CAN_ID || header.StdId > CAN_ID + PARAM_SIZE)
		return;

	memcpy(CAN_PACKETS[header.StdId - CAN_ID], &data, CAN_PACKETS_SIZE[header.StdId - CAN_ID]);
}

