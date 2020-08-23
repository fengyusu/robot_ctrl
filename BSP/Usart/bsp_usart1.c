/**
  ******************************************************************************
  * @file    bsp_usart1.c
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   用于接收遥控器数据，连接接收机，pb7--usart1_rx
  */ 
  
#include "bsp_usart1.h"
#include "Remote_Task.h"
#include "fifo.h"



//************************************************************************************
//!                                  VARIABLES
//************************************************************************************
static uint8_t USART1_DMA_RX_BUF[BSP_USART1_DMA_RX_BUF_LEN];


static void USART1_FIFO_Init(void);

 /**
  * @brief  USART1 GPIO 配置,工作模式配置。baud_rate 8-N-1
  * @param  baud_rate :串口波特率
  * @retval 无
  */
void USART1_Configuration(uint32_t baud_rate)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;
    DMA_InitTypeDef dma;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);//只需要接收端
    
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);
    
    USART_DeInit(USART1);
    USART_StructInit(&usart);
    usart.USART_BaudRate = baud_rate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_Even;
    usart.USART_Mode = USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart);
    
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    
    DMA_DeInit(DMA2_Stream2);
    DMA_StructInit(&dma);
    dma.DMA_Channel = DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)&USART1_DMA_RX_BUF[0];
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = sizeof(USART1_DMA_RX_BUF)/2;              //?
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_Medium;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;                //禁用fifo
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;           //单独模式（不使能突发模式）
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   //单独模式（不使能突发模式）
    DMA_Init(DMA2_Stream2, &dma);
    
    DMA_Cmd(DMA2_Stream2, ENABLE);
    
    nvic.NVIC_IRQChannel = USART1_IRQn;                          
    nvic.NVIC_IRQChannelPreemptionPriority = 4;   //pre-emption priority 
    nvic.NVIC_IRQChannelSubPriority = 0;		    //subpriority 
    nvic.NVIC_IRQChannelCmd = ENABLE;			
    NVIC_Init(&nvic);	
    
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);        //usart bus idle interrupt  enabled
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	static uint32_t this_time_rx_len = 0;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		//clear the idle pending flag 
		(void)USART1->SR;
		(void)USART1->DR;

		//Target is Memory0
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0)
		{
            //切换缓冲区为memory1
			DMA_Cmd(DMA2_Stream2, DISABLE);
			this_time_rx_len = BSP_USART1_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA_SetCurrDataCounter(DMA2_Stream2,BSP_USART1_DMA_RX_BUF_LEN);    

            if(this_time_rx_len == RC_FRAME_LENGTH)
			{
				//RemoteDataPrcess(USART1_DMA_RX_BUF);
				xQueueSendFromISR(remoteQueue_Get(), (void *)USART1_DMA_RX_BUF, &xHigherPriorityTaskWoken);
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			}
			
			DMA_Cmd(DMA2_Stream2, ENABLE);
		}

	}       
}




/*********************************************END OF FILE**********************/
