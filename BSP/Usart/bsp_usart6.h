/**
* @file     bsp_usart6.h
* @brief    usart6 head file
* @author   Hsc
* @date     2015-10-23
* @version  1.0
* @license  Copyright(c) 2015 DJI Corporation Robomasters Department. 
*/

#ifndef __BSP_USART6_H
#define __BSP_USART6_H


/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "stm32f4xx.h"

/*
*********************************************************************************************************
*                                      FUNCTION PROTOTYPES
*********************************************************************************************************
*/
void USART6_Configuration(u32 baud);
void UART6_PrintCh(uint8_t ch);
void UART6_PrintBlock(uint8_t* pdata, uint8_t len);

#endif



