#ifndef __BSP_TIMER1_H__
#define __BSP_TIMER1_H__

#include "stm32f4xx.h"

#define PWM1  TIM12->CCR1
#define PWM2  TIM12->CCR2


#define SetIRFreq(x)   (PWM1 = x);                          

#define InitFrictionWheel()     \
        PWM1 = 1000;             \
        PWM2 = 1000;
		
#define SetFrictionWheelSpeed(x) \
        PWM1 = x;                \
        PWM2 = x;


void TIM1_Configuration(void);





#endif
/*********************************************** end of bsp_tim5.h **************************************************/

