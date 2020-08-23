#include "bsp_usart3.h"
#include "fifo.h"
#include <stdint.h>
#include "includes.h"

/*-----USART3_TX-----PD8-----*/
/*-----USART3_RX-----PD9-----*/

FIFO_S_t* UART3_TranFifo;

void USART3_Configuration(uint32_t baud)
{
    //GPIO端口设置
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart;
	NVIC_InitTypeDef nvic;



	RCC_AHB1PeriphClockCmd(RCC_USART3_GPIO, ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_USART3_Periph, ENABLE); 

	GPIO_PinAFConfig(GPIO_USART3, PIN_SOURCE_USART3_RX, GPIO_USART3_AF);
	GPIO_PinAFConfig(GPIO_USART3, PIN_SOURCE_USART3_TX, GPIO_USART3_AF); 

	gpio.GPIO_Pin = GPIO_PIN_USART3_RX | GPIO_PIN_USART3_TX;
	gpio.GPIO_Mode = GPIO_Mode_AF;                                      //复用功能
	gpio.GPIO_Speed = GPIO_Speed_50MHz;	                                //速度50MHz
	gpio.GPIO_OType = GPIO_OType_PP;                                    //推挽复用输出
	gpio.GPIO_PuPd = GPIO_PuPd_UP;                                      //上拉
	GPIO_Init(GPIO_USART3, &gpio); 

	usart.USART_BaudRate = baud;                                        //波特率设置
	usart.USART_WordLength = USART_WordLength_8b;                       //字长为8位数据格式
	usart.USART_StopBits = USART_StopBits_1;                            //一个停止位
	usart.USART_Parity = USART_Parity_No;                               //无奇偶校验位
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //无硬件数据流控制
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                //收发模式
	USART_Init(USART3, &usart);                                          //初始化串口3


	nvic.NVIC_IRQChannel = USART3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 6;            //抢占优先级
	nvic.NVIC_IRQChannelSubPriority = 0;		            //子优先级
	nvic.NVIC_IRQChannelCmd = ENABLE;			                        //IRQ通道使能
	NVIC_Init(&nvic);	                                                

    USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);                        //接收寄存器非空中断
	USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);
    USART_Cmd(USART3,ENABLE);                                           //使能串口3
        
	UART3_TranFifo = FIFO_S_Create(256);                                          
    if(!UART3_TranFifo)
    {
       while(1);  
	}
}


void UART3_PrintCh(uint8_t ch)
{    
    FIFO_S_Put(UART3_TranFifo, ch);
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    
}

void UART3_PrintBlock(uint8_t* pdata, uint8_t len)
{
	uint8_t i = 0;
    for(i = 0; i < len; i++)
    {
        FIFO_S_Put(UART3_TranFifo, pdata[i]);
    }
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);  //发送寄存器空中断
}


int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
    USART_SendData(USART3, (uint8_t)ch);
    return ch;
}

void USART3_IRQHandler(void)
{  
    if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)        //发送寄存器空中断
    {   
        if(!FIFO_S_IsEmpty(UART3_TranFifo))
        {
            uint16_t dataSend = (uint16_t)FIFO_S_Get(UART3_TranFifo);
            USART_SendData(USART3, dataSend);
        }
        else
        {
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
        }  
		USART_ClearITPendingBit(USART3, USART_IT_TXE);
    }
    else if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)    //接收寄存器空中断
    {       
		USART_ClearFlag(USART3,USART_FLAG_RXNE)	;
		USART_ClearITPendingBit(USART3,USART_IT_RXNE); 
		
		uint8_t dataReceive = USART_ReceiveData(USART3);     
		FIFO_S_Put(GetUsart3RxFifo(),dataReceive);

    }      
    else if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)    //接收寄存器空中断
    {
        (void)USART3->SR;
		(void)USART3->DR;

		BaseType_t xHighPriorityTaskWoken = pdFALSE;
		BaseType_t xResult = xEventGroupSetBitsFromISR(xGlobalEventGroup, EVENT_USART3_RECEIVE, &xHighPriorityTaskWoken);
		if(xResult != pdFALSE)
		{
			portYIELD_FROM_ISR(xHighPriorityTaskWoken);
		}
    }	
}

