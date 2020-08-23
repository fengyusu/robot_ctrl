#include "includes.h"

EventGroupHandle_t xGlobalEventGroup = NULL;
xTaskHandle xHandleTask_LED_R;
xQueueHandle xQueue;

int main()
{	

	bsp_init();
	
	xGlobalEventGroup = xEventGroupCreate();
	
	xQueue = xQueueCreate(5, sizeof(uint8_t));	
	
	xTaskCreate(LED_R_Task,       "LED_R",    configMINIMAL_STACK_SIZE,     NULL, 1, &xHandleTask_LED_R);
	xTaskCreate(LED_G_Task,       "LED_G",    configMINIMAL_STACK_SIZE,     NULL, 1, NULL);
	
	xTaskCreate(IMU_Task,         "IMU",      configMINIMAL_STACK_SIZE*2,   NULL, 1, NULL);
	xTaskCreate(GimabalCtrl_Task, "GIMBAL",   configMINIMAL_STACK_SIZE*3,   NULL, 3, NULL);
	xTaskCreate(ShooterCtrl_Task, "SHOOTER",  configMINIMAL_STACK_SIZE*3,   NULL, 3, NULL);
	xTaskCreate(Remote_Task,      "REMOTE",   configMINIMAL_STACK_SIZE*3,   NULL, 4, NULL);
	xTaskCreate(Protocol_Task,    "PROTOCOL", configMINIMAL_STACK_SIZE*3,   NULL, 4, NULL);
	
	//xTaskCreate(DataSend_Task, "DATA_SEND", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//	xTaskCreate(Print_SysInfo_Task, "SYS_INFO", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	
	vTaskStartScheduler();
	
	while(1)
	{
        
	}
}





void vApplicationIdleHook(void)
{

}

void vApplicationTickHook(void)
{
#if ( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY == 0 )
    {
        // In this case the tick hook is used as part of the queue set test. 
    }
#endif // mainCREATE_SIMPLE_BLINKY_DEMO_ONLY 
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for(;;);
}

void vApplicationStackOverflowHook(void)
{

}
