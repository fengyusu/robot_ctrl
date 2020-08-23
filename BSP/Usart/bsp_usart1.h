#ifndef __USART1_H
#define	__USART1_H

#include "stm32f4xx.h"

//************************************************************************************
//!                                     MACROS
//************************************************************************************

#define  BSP_USART1_DMA_RX_BUF_LEN               20u     
#define  RC_FRAME_LENGTH                         18u

/*
*********************************************************************************************************
*                                             FUNCTION PROTOTYPES
*********************************************************************************************************
*/
void USART1_IRQHandler(void);
void USART1_Configuration(uint32_t baud_rate);
void RemoteDataPrcess(uint8_t *pData);


#endif /* __USART1_H */
