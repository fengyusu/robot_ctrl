#ifndef __CAN1_H__
#define __CAN1_H__

#include "stm32f4xx_conf.h"
#include "fifo.h"

/************************* Commond List start*******************************/



void CAN1_ConfigInit(void);
void CAN1_Receive_Enable(void);
void Can_Send_StdMsg(uint16_t stdid, uint8_t* msg, uint8_t len);
void CAN1_SendBlock(uint16_t stdid, uint8_t* pdat, uint16_t len);






#endif //__CAN_H__

