/**
* @file     bsp_usart6.c
* @brief    usart6 implement
* @author   Hsc
* @date     2015-10-23
* @version  1.0
* @license  Copyright(c) 2015 DJI Corporation Robomasters Department. 
*/



/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "bsp_usart6.h"
#include "fifo.h"
#include <stdio.h>
#include "includes.h"

/*
*********************************************************************************************************
*                                      FUNCTION PROTOTYPES
*********************************************************************************************************
*/
           
FIFO_S_t* UART6_TranFifo;

/**
* @fn       BSP_USART6_Init
* @brief    ���ڳ�ʼ��
* @param    baud��������
* @retval   ��
* @note     ��ʽ��8-E-1
*/
void USART6_Configuration(u32 baud) 
{
	//GPIO�˿�����
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart;
	NVIC_InitTypeDef nvic;
	USART_DeInit(USART6);
	
    UART6_TranFifo = FIFO_S_Create(256);  
    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); 

	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6); 

	gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
	gpio.GPIO_Mode = GPIO_Mode_AF;                                      //���ù���
	gpio.GPIO_Speed = GPIO_Speed_50MHz;	                                //�ٶ�50MHz
	gpio.GPIO_OType = GPIO_OType_PP;                                    //���츴�����
	gpio.GPIO_PuPd = GPIO_PuPd_UP;                                      //����
	GPIO_Init(GPIOG, &gpio); 
  

	usart.USART_BaudRate = baud;                                        //����������
	usart.USART_WordLength = USART_WordLength_8b;                       //�ֳ�Ϊ8λ���ݸ�ʽ
	usart.USART_StopBits = USART_StopBits_1;                            //һ��ֹͣλ
	usart.USART_Parity = USART_Parity_No;                               //����żУ��λ
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //��Ӳ������������
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                //�շ�ģʽ
	USART_Init(USART6, &usart); 

	nvic.NVIC_IRQChannel = USART6_IRQn;                                 //����3�ж�ͨ��
	nvic.NVIC_IRQChannelPreemptionPriority = 5;           //��ռ���ȼ�
	nvic.NVIC_IRQChannelSubPriority = 0;		            //�����ȼ�
	nvic.NVIC_IRQChannelCmd = ENABLE;			                        //IRQͨ��ʹ��
	NVIC_Init(&nvic);	                                                //����ָ���Ĳ�����ʼ��VIC�Ĵ�����
    

	USART_ClearFlag(USART6, USART_FLAG_TC);
	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);
	USART_ITConfig(USART6,  USART_IT_RXNE, ENABLE);//��������ж�
	USART_Cmd(USART6, ENABLE);
    
}

void UART6_PrintCh(uint8_t ch)
{    
    FIFO_S_Put(UART6_TranFifo, ch);
    USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
    
}

void UART6_PrintBlock(uint8_t* pdata, uint8_t len)
{
	uint8_t i = 0;
    for(i = 0; i < len; i++)
    {
        FIFO_S_Put(UART6_TranFifo, pdata[i]);
    }
    USART_ITConfig(USART6, USART_IT_TXE, ENABLE);  //���ͼĴ������ж�
}


void USART6_IRQHandler(void)
{  
 
    if(USART_GetITStatus(USART6, USART_IT_TXE) != RESET)        //���ͼĴ������ж�
    {
        if(!FIFO_S_IsEmpty(UART6_TranFifo))
        {
            uint16_t data = (uint16_t)FIFO_S_Get(UART6_TranFifo);
            USART_SendData(USART6, data);
        }
        else
        {
            USART_ITConfig(USART6, USART_IT_TXE, DISABLE);      //������ʱ���رշ����ж�
        } 
        USART_ClearITPendingBit(USART6, USART_IT_TXE);		
    }
	else if (USART_GetITStatus(USART6, USART_IT_RXNE) == SET)   
	{ 
        USART_ClearFlag(USART6,USART_FLAG_RXNE)	;
		USART_ClearITPendingBit(USART6,USART_IT_RXNE);        
        
		uint8_t data = USART_ReceiveData(USART6);     
		FIFO_S_Put(GetUsart6RxFifo(),data);

    }
	else if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)    //���ռĴ������ж�
    {
        (void)USART6->SR;
		(void)USART6->DR;

		BaseType_t xHighPriorityTaskWoken = pdFALSE;
		BaseType_t xResult = xEventGroupSetBitsFromISR(xGlobalEventGroup, EVENT_USART6_RECEIVE, &xHighPriorityTaskWoken);
		if(xResult != pdFALSE)
		{
			portYIELD_FROM_ISR(xHighPriorityTaskWoken);
		}
    }
}
