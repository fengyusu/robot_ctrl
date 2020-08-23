#include "bsp_can1.h"
#include "GimbalCtrl_Task.h"
#include "ShooterCtrl_Task.h"
#include <string.h>



/**
  * @brief  配置CAN1
  * @param  None
  * @retval None
  */
void CAN1_ConfigInit(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOD, &gpio);
    
    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);    
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = ENABLE;
    can.CAN_RFLM = ENABLE;
    can.CAN_TXFP = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN1, &can);

	can_filter.CAN_FilterNumber=0;
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;
	can_filter.CAN_FilterMaskIdLow=0x0000;
	can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN1,CAN_IT_FMP0,DISABLE);
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}

void CAN1_Receive_Enable(void)
{
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
}

/**
  * @brief  Judge whther a mail box is empty
  * @param  mailBoxIndex : the number of mailbox ;this param can be 0~2;
  * @retval flag：
                1 is empty;
                0 is not empty.
  */
static uint8_t CAN1ExistEmptMailBox(uint8_t mailBoxIndex)
{
    uint8_t flag = 0;
    switch (mailBoxIndex)
    {
        case 0:
            if((CAN1->TSR & CAN_TSR_TME0) == 0) flag = 0;
            else flag = 1;
            break;
        case 1:
            if((CAN1->TSR & CAN_TSR_TME1) == 0) flag = 0;
            else flag = 1;
            break;
        case 2:
            if((CAN1->TSR & CAN_TSR_TME2) == 0) flag = 0;
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
void Can_Send_StdMsg(uint16_t stdid, uint8_t* msg, uint8_t len)
{ 
    CanTxMsg TxMessage;

    //等待发送邮箱空
    while(CAN1ExistEmptMailBox(0) == 0);

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
    CAN_Transmit(CAN1, &TxMessage); 
}

void CAN1_SendBlock(uint16_t stdid, uint8_t* pdat, uint16_t len)
{
    uint16_t tmpLen_p, tmpLen_r;
    uint16_t i;

    tmpLen_p    = len / 8;
    tmpLen_r    = len % 8;

    for(i=0; i < tmpLen_p; i++)
    {
        Can_Send_StdMsg(stdid, pdat, 8);
        pdat += 8;
    }

    if(tmpLen_r != 0)
    {
        Can_Send_StdMsg(stdid, (pdat), tmpLen_r);
    }

}

void CAN1_SendBuff(uint16_t stdid, FIFO_S_t *fifo)
{

}






void CAN1_TX_IRQHandler(void)
{
    if(CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
    {
         CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
    }
}

void CAN1_RX0_IRQHandler(void)
{   
    CanRxMsg rx_message;
    if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 
    
        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
        Can1_ReceiveGimbalMsgProcess(&rx_message);
        Can1_ReceiveShooterMsgProcess(&rx_message);
		
		Can2_ReceiveGyroMsgProcess(&rx_message);
    }
}


