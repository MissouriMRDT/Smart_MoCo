#include "smoco.h"
#include "main.h"
#include <memory.h>
#include <stdio.h>

#define CAN_ID 0

static uint8_t* CAN_PACKETS[PARAM_SIZE] = {NULL};

void INIT_PACKET_ORG()
{
	if (CAN_PACKETS[0] == NULL)
	{
		CAN_PACKETS[0] = (uint8_t*)&(params.rPosition);
		CAN_PACKETS[1] = (uint8_t*)&(params.feedForwardP);
		CAN_PACKETS[2] = (uint8_t*)&(params.feedForwardV);
		CAN_PACKETS[3] = (uint8_t*)&(params.feedForwardC);

		CAN_PACKETS[4] = (uint8_t*)&(params.p);
		CAN_PACKETS[5] = (uint8_t*)&(params.d);

		CAN_PACKETS[6] = (uint8_t*)&(params.le);
		CAN_PACKETS[7] = (uint8_t*)&(params.alim);

		CAN_PACKETS[8] = (uint8_t*)&(params.rampRate);
		CAN_PACKETS[9] = (uint8_t*)&(params.fwdMax);
		CAN_PACKETS[10] = (uint8_t*)&(params.dutyCycle);

		CAN_PACKETS[11] = (uint8_t*)&(params.calibrationDutyCycle);
	}
}

const uint8_t CAN_PACKETS_SIZE[PARAM_SIZE] =
{
	7,
	6,
	6,
	4,

	8,
	4,

	4,
	8,

	4,
	8,
	2,

	6
};
