#ifndef SMOCO.H
#define SMOCO.H

#include <stdbool.h>
#include <stdint.h>

#define PARAM_SIZE 12

typedef struct __attribute__((__packed__))
{
	int32_t rPosition;
	int16_t rVelocity;
	uint8_t rCurrent;

	float p;
	float i;

	float d;

	uint8_t le;

	int32_t alim;
	int32_t blim;

	float rampRate;

	uint16_t calibrationDutyCycle;
	uint32_t calibrationPosition;

	int16_t fwdMax;
	int16_t fwdMin;
	int16_t revMin;
	int16_t revMax;

	uint16_t dutyCycle;

	int16_t feedForwardP;
	int32_t tPosition;

	int16_t feedForwardV;
	float tVelocity;

	int16_t feedForwardC;
	int16_t tCurrent;
} SMOCOd;

static SMOCOd params;

#endif

