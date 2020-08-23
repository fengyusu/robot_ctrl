#include "bsp_timer2.h"


void TIM2_Configuration(void)
{

    TIM_TimeBaseInitTypeDef    tim;
    NVIC_InitTypeDef           nvic;


    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);


    tim.TIM_Prescaler = 90-1;        //90M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 26;  //1ms,1000Hz
    TIM_TimeBaseInit(TIM2,&tim);
	

	
	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority=7;//抢占优先级1
	nvic.NVIC_IRQChannelSubPriority =0;		//子优先级
	nvic.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&nvic);	//初始化NVIC寄存器
	
	TIM_Cmd(TIM2, ENABLE);	 
    TIM_ITConfig(TIM2, TIM_IT_Update,ENABLE);

}
