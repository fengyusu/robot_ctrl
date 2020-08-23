#include "includes.h"
 


void LED_R_Task(void* param)
{
    while(1)
	{
	    RED_LED_ON();
		vTaskDelay(1000);
		RED_LED_OFF();
		vTaskDelay(500);
		
	}
}

void LED_G_Task(void* param)
{
    while(1)
	{
	    GREEN_LED_TOGGLE();
		vTaskDelay(1000);
	}
}