#include "bsp_init.h"
#include "delay.h"

void bsp_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
    Led_Configuration();
	ShootKey_Configuration();
	CAN1_ConfigInit();	
	CAN2_ConfigInit();
    TIM6_Configuration();
	TIM6_Start();
	TIM12_Configuration();
	USART3_Configuration(115200);  
	USART1_Configuration(100000);  

}

