#include "can.h"

void InitSmocoCan(CAN_HandleTypeDef *hcan)
{
	INIT_PACKET_ORG();//this is needed to write the data from the packets.

	CAN_FilterTypeDef filter;

	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.FilterBank = 0;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterIdHigh = 0;//disable filtering for now.
	filter.FilterMaskIdHigh = 0;
	filter.FilterIdLow = 0;
	filter.FilterMaskIdLow = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(hcan, &filter);
	HAL_CAN_Start(hcan);//Restart/Start CAN.
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t data[8];
	CAN_RxHeaderTypeDef header;

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
		return;

	if (header.StdId < CAN_ID || header.StdId > CAN_ID + PARAM_SIZE)
		return;

	memcpy(CAN_PACKETS[header.StdId - CAN_ID], data, CAN_PACKETS_SIZE[header.StdId - CAN_ID]);
}

void SmocoPushTXBuffer(CAN_HandleTypeDef *hcan, SMOCO_PACKET_TYPE p)
{
	CAN_TxHeaderTypeDef header;
	uint8_t data[8] = {0};
	uint32_t mailbox;

	header.IDE = CAN_ID_STD;
	header.StdId = CAN_ID + p;
	header.RTR = CAN_RTR_DATA;
	header.DLC = CAN_PACKETS_SIZE[p];

	memcpy(data, CAN_PACKETS[p], CAN_PACKETS_SIZE[p]);

	HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}
