#include "bsp_timer6.h"

volatile unsigned long ulHighFrequencyTimerTicks;

void TIM6_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    
    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 8;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 90-1;        //90M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 1000;  //1ms,1000Hz
    TIM_TimeBaseInit(TIM6,&tim);
	
	TIM_Cmd(TIM6, DISABLE);	 
    TIM_ITConfig(TIM6, TIM_IT_Update,DISABLE);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
}

void TIM6_Start(void)
{
    TIM_Cmd(TIM6, ENABLE);	 
    TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
}

void TIM6_Stop(void)
{
    TIM_Cmd(TIM6, DISABLE);	 
    TIM_ITConfig(TIM6, TIM_IT_Update,DISABLE);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
}

void TIM6_DAC_IRQHandler(void)  
{	
    if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
        TIM_ClearFlag(TIM6, TIM_FLAG_Update);
        
		ulHighFrequencyTimerTicks++;
    }
}


