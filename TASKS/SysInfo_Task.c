#include "includes.h"
 

int8_t pcWriteBuffer[500];

void Print_SysInfo_Task(void* param)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 1000;

	 /* 获取系统时间 */
     xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        vTaskList((char *)&pcWriteBuffer);
		printf("%s\r\n", pcWriteBuffer);
		
		vTaskGetRunTimeStats((char *)&pcWriteBuffer);
		printf("%s\r\n", pcWriteBuffer);
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}