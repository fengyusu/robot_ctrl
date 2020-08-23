#ifndef __BSP_TIMER6_H__
#define __BSP_TIMER6_H__

#include "stm32f4xx.h"
#include "FreeRTOS.h"

void TIM6_Configuration(void);
void TIM6_Start(void);
void TIM6_Stop(void);

#endif
