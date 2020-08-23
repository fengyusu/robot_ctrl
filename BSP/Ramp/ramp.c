#include "ramp.h"

void RampInit(RampGen_t *ramp, int32_t XSCALE)
{
	ramp->count = 0;
	ramp->XSCALE = XSCALE;
}

float RampCalc(RampGen_t *ramp)
{
	if(ramp->XSCALE <= 0)
	{
		return 0;
	}
		
	if(ramp->count++ < ramp->XSCALE)
	{		
	}
	else
	{
		ramp->count = ramp->XSCALE;
	}
	ramp->out = ramp->count / ((float)ramp->XSCALE); 
	return ramp->out;
}

void RampSetCounter( RampGen_t *ramp, int32_t count)
{
	ramp->count = count;
}

void RampResetCounter( RampGen_t *ramp)
{
	ramp->count = 0;
}
void RampSetScale( RampGen_t *ramp, int32_t scale)
{
	ramp->XSCALE = scale;
}
uint8_t RampIsOverflow(RampGen_t *ramp)
{
	if(ramp->count >= ramp->XSCALE)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

