#include "smoco.h"

#define CAN_ID 0

static const uint8_t* CAN_PACKETS[PARAM_SIZE] =
{
	&(params.rPosition),
	&(params.feedForwardP),
	&(params.feedForwardV),
	&(params.feedForwardC),

	&(params.p),
	&(params.d),

	&(params.le),
	&(params.alim),

	&(params.rampRate),
	&(params.fwdMax),
	&(params.dutyCycle),

	&(params.calibrationDutyCycle)
};

static const uint8_t CAN_PACKETS_SIZE[PARAM_SIZE] =
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
