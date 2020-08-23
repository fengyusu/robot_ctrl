#ifndef __CAN2_H__
#define __CAN2_H__
#include <stdint.h>


void CAN2_ConfigInit(void);
void CAN2_SendBlock(uint16_t stdid, uint8_t* pdat, uint16_t len);


#endif 
