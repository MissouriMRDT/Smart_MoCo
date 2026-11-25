#include "smoco.h"
#include "main.h"
#include <memory.h>
#include <stdio.h>

#define CAN_ID 0

static uint8_t* CAN_PACKETS[PARAM_SIZE] = {NULL};

typedef enum {
	SMOCO_PVC,
	SMOCO_TARGET_POSITION,
	SMOCO_TARGET_VELOCITY,
	SMOCO_TARGET_CURRENT,
	SMOCO_PI,
	SMOCO_D,
	SMOCO_LIMIT_ENABLE,
	SMOCO_LIMIT_POSITIONS,
	SMOCO_RAMP,
	SMOCO_DUTY_RANGE,
	SMOCO_DUTY,
	SMOCO_CALIBRATE
} SMOCO_PACKET_TYPE;

void INIT_PACKET_ORG()
{
	if (CAN_PACKETS[0] == NULL)
	{
		CAN_PACKETS[SMOCO_PVC] = (uint8_t*)&(params.rPosition);
		CAN_PACKETS[SMOCO_TARGET_POSITION] = (uint8_t*)&(params.feedForwardP);
		CAN_PACKETS[SMOCO_TARGET_VELOCITY] = (uint8_t*)&(params.feedForwardV);
		CAN_PACKETS[SMOCO_TARGET_CURRENT] = (uint8_t*)&(params.feedForwardC);

		CAN_PACKETS[SMOCO_PI] = (uint8_t*)&(params.p);
		CAN_PACKETS[SMOCO_D] = (uint8_t*)&(params.d);

		CAN_PACKETS[SMOCO_LIMIT_ENABLE] = (uint8_t*)&(params.le);
		CAN_PACKETS[SMOCO_LIMIT_POSITIONS] = (uint8_t*)&(params.alim);

		CAN_PACKETS[SMOCO_RAMP] = (uint8_t*)&(params.rampRate);
		CAN_PACKETS[SMOCO_DUTY_RANGE] = (uint8_t*)&(params.fwdMax);
		CAN_PACKETS[SMOCO_DUTY] = (uint8_t*)&(params.dutyCycle);

		CAN_PACKETS[SMOCO_CALIBRATE] = (uint8_t*)&(params.calibrationDutyCycle);
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
