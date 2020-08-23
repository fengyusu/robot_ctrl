#include "bsp_can2.h"
#include "stm32f4xx_conf.h"
#include <string.h>
#include "GimbalCtrl_Task.h"
/*----CAN2_TX-----PB13----*/
/*----CAN2_RX-----PB12----*/


void CAN2_ConfigInit(void)
{
    CAN_InitTypeDef        can2;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使用can2必须把can1时钟也使能
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	gpio.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	gpio.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB, &gpio); 

    nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    CAN_DeInit(CAN2);
    CAN_StructInit(&can2);

    //config
	can2.CAN_TTCM      = DISABLE;
	can2.CAN_ABOM      = ENABLE;
	can2.CAN_AWUM      = DISABLE;
	can2.CAN_NART      = ENABLE;
	can2.CAN_RFLM      = ENABLE;				
	can2.CAN_TXFP      = ENABLE;		
	can2.CAN_Mode      = CAN_Mode_LoopBack;
	can2.CAN_SJW       = CAN_SJW_1tq;
	can2.CAN_BS1       = CAN_BS1_5tq;
	can2.CAN_BS2       = CAN_BS2_3tq;
	can2.CAN_Prescaler = 5;     //CAN BaudRate 45/(1+5+3)/5=1Mbps
	CAN_Init(CAN2, &can2);
    
    can_filter.CAN_FilterNumber=14; //CAN2筛选器组范围：14-27
    can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh=0x0000;
    can_filter.CAN_FilterIdLow=0x0000;
    can_filter.CAN_FilterMaskIdHigh=0x0000;
    can_filter.CAN_FilterMaskIdLow=0x0000;
    can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
    can_filter.CAN_FilterActivation=ENABLE;    
    CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
    CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
}


static uint8_t CAN2ExistEmptMailBox(uint8_t mailBoxIndex)
{
    uint8_t flag = 0;
    switch (mailBoxIndex)
    {
        case 0:
            if((CAN2->TSR & CAN_TSR_TME0) == 0) flag = 0;
            else flag = 1;
            break;
        case 1:
            if((CAN2->TSR & CAN_TSR_TME1) == 0) flag = 0;
            else flag = 1;
            break;
        case 2:
            if((CAN2->TSR & CAN_TSR_TME2) == 0) flag = 0;
            else flag = 1;
            break;
    } 
    return flag;
} 

/**
  * @brief  CAN1发送消息
  * @param  stdid 标准ID
  * @param  *msg  消息指针
  * @param  len   数据长度
  * @retval None
  */
void Can2_Send_StdMsg(uint16_t stdid, uint8_t* msg, uint8_t len)
{ 
    CanTxMsg TxMessage;

    //等待发送邮箱空
    while(CAN2ExistEmptMailBox(0) == 0);

    TxMessage.StdId = stdid;	     
    TxMessage.ExtId = 0;	            
    TxMessage.IDE   = CAN_Id_Standard; 
    TxMessage.RTR   = CAN_RTR_Data;    
    TxMessage.DLC   = len;    	     

    memcpy(TxMessage.Data, msg, len);
    /*
    函数原型 :void *memcpy(void *dest, const void *src, size_t n);
    功能: 从源src所指的内存地址的起始位置开始拷贝n个字节到目标dest所指的内存地址的起始位置中
    */
    CAN_Transmit(CAN2, &TxMessage); 
}

void CAN2_SendBlock(uint16_t stdid, uint8_t* pdat, uint16_t len)
{
    uint16_t tmpLen_p, tmpLen_r;
    uint16_t i;

    tmpLen_p    = len / 8;
    tmpLen_r    = len % 8;

    for(i=0; i < tmpLen_p; i++)
    {
        Can2_Send_StdMsg(stdid, pdat, 8);
        pdat += 8;
    }

    if(tmpLen_r != 0)
    {
        Can2_Send_StdMsg(stdid, (pdat), tmpLen_r);
    }

}


void CAN2_TX_IRQHandler(void) //CAN TX
{
  if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET)    //if transmit mailbox is empty 
  {
	   CAN_ClearITPendingBit(CAN2,CAN_IT_TME);   
  }
}



void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx_message);  
       //电机编码器数据处理
        Can2_ReceiveGyroMsgProcess(&rx_message);
    }
}

