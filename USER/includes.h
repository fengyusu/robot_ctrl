#ifndef __INCLUDES_H__
#define __INCLUDES_H__

#include <stdio.h>
#include <string.h>

 // Kernel includes. 
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"

#include "stm32f4xx_conf.h"

#include "bsp_init.h"
#include "pid_regulator.h"
#include "ramp.h"
#include "param.h"

#include "Led_Task.h"
#include "IMU_Task.h"
#include "DataSend_Task.h"
#include "SysInfo_Task.h"
#include "GimbalCtrl_Task.h"
#include "ShooterCtrl_Task.h"
#include "Remote_Task.h"
#include "Protocol_Task.h"

extern EventGroupHandle_t xGlobalEventGroup;

#endif
