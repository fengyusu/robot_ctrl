#include "includes.h"
#include "fifo.h"
#include "MF_CRC16.h" 
#include "bsp_can1.h"
 
 
static tModuleCmdFunctionList gModuleCmdFunctionTable[MAX_CMD_NUM];   
 
static tUnpackObj gCan1UnpackObj;
static uint8_t gCan1RxBuffer[CAN_RX_BUF_SIZE];
static FIFO_S_t Can1RxFifo;
static uint8_t gCan1TxBuffer[CAN_TX_BUF_SIZE];
static FIFO_S_t Can1TxFifo;

static tUnpackObj gUsart3UnpackObj;
static uint8_t gUsart3RxBuffer[USART3_RX_BUF_SIZE];
static FIFO_S_t Usart3RxFifo;

static tUnpackObj gUsart6UnpackObj;
static uint8_t gUsart6RxBuffer[USART6_RX_BUF_SIZE];
static FIFO_S_t Usart6RxFifo;
 
FIFO_S_t * GetCan1RxFifo(void)
{
    return &Can1RxFifo;
}

FIFO_S_t * GetCan1TxFifo(void)
{
    return &Can1TxFifo;
}

FIFO_S_t * GetUsart3RxFifo(void)
{
    return &Usart3RxFifo;
}

FIFO_S_t * GetUsart6RxFifo(void)
{
    return &Usart6RxFifo;
}

void Potocol_Transmit(uint8_t  send_type,    uint8_t send_id,    \
	                     uint8_t  recieve_type, uint8_t recieve_id, \
                         uint16_t cmd_id,        uint8_t* pdat,     uint16_t len)
{
    static uint8_t txBuf[128]={0};
    static uint16_t m_seq_number = 0;
    tHeader *pHeader = (tHeader *)txBuf;
    uint8_t headsize;
    uint8_t dataSize;
    uint32_t frameSize;

    headsize = sizeof(tHeader);
    dataSize = len;
    frameSize = headsize + dataSize + 2; //帧头+数据+CRC16

    pHeader->sof         = PROTOCAL_HEADER;
    pHeader->length      = dataSize;
    pHeader->seq_num     = m_seq_number++;

    pHeader->sendType    = send_type;
    pHeader->sendID      = send_id;
    pHeader->receiveType = recieve_type;
    pHeader->receiveID   = recieve_id;
	
    pHeader->cmd_id      = cmd_id;
    Append_CRC8_Check_Sum(txBuf, headsize);

    //将数据拷贝到内存中
    memcpy(txBuf + headsize, pdat, dataSize);
    //将CRC16加入内存中
    Append_CRC16_Check_Sum(txBuf,frameSize);

    switch(recieve_type)
	{
		case MODULE_TYPE_CAR_CHASSIS:
		{
		    CAN1_SendBlock(CAN_ID_CAR_GIMBAL, txBuf, frameSize);
			CAN2_SendBlock(CAN_ID_CAR_GIMBAL, txBuf, frameSize);
		}break;
		
		case MODULE_TYPE_CAR_GIMBAL:
		{
		
		}break;
		
		case MODULE_TYPE_MINI_PC:
		{
		    UART6_PrintBlock(txBuf, frameSize);
		}break;
		
		case MODULE_TYPE_PC:
		{
		    UART3_PrintBlock(txBuf, frameSize);
		}break;
		
		case ALL_MODULE:
		{
		
		}break;
		
		default:
			break;

	}
}

/**
*@func:     ModuleCmdIdFuncTableInit
*@brief:    init the table of protocol cmdid handler function
*@param:    none
*@ret:      none
*/
void  ModuleCmdIdFuncTableInit(void)
{
    int i;
    for(i = 0 ; i < MAX_CMD_NUM;i++)
    {
        gModuleCmdFunctionTable[i].cmd_id=NULL;
        gModuleCmdFunctionTable[i].handler=NULL;
    }
}

/**
*@func:     ProPackageHandleFuncRegister
*@brief:    register cmd handler function in cmd id handler table(gModuleCmdFunctionTable)
*@param:    none
*@note:     you must init the gModuleCmdFunctionTable through calling "ModuleCmdIdFuncTableInit()"
*           before registering your cmdid hander function
*@ret:      0: success
*           -1:failed
*/
int ProPackageHandleFuncRegister(uint16_t cmd_id,CanCmdFuncHandle fun_ptr)
{
    int i;
    for(i = 0;i < MAX_CMD_NUM; i++)
    {
        if((gModuleCmdFunctionTable[i].cmd_id == NULL) && (gModuleCmdFunctionTable[i].handler == NULL))
        {
            gModuleCmdFunctionTable[i].cmd_id=cmd_id;
            gModuleCmdFunctionTable[i].handler=fun_ptr;
            return 0;
        }
    }
    return -1;   // if  run here, it indicate the register fail.
}

/**
*@func:     ModuleCmdIdFuncTableInit
*@brief:    init the table of protocol cmdid handler function
*@param:    none
*@author:   damom.li
*/
static void ProPackageHandle(uint8_t* p_buf, uint16_t len)
{
    tHeader* p_cmd= (tHeader*) p_buf;
    uint8_t i=0;

    if((p_cmd->receiveType == ALL_MODULE) ||(p_cmd->receiveType == MODULE_TYPE))
    {
        for( i = 0; (gModuleCmdFunctionTable[i].handler != NULL) && (i < MAX_CMD_NUM); i++ )
        {
            if( p_cmd->cmd_id == gModuleCmdFunctionTable[i].cmd_id )
            {
                gModuleCmdFunctionTable[i].handler( p_buf, len);
//                printf("exe id: 0x%x\r\n", p_cmd->cmd_id);
            }
        }
    }
}


/* PC and module communication */
 void  Forward_Packet(uint8_t* p_buf,uint16_t len)
{

    tHeader* theader_ptr = (tHeader*)p_buf;
    uint8_t headerLen = sizeof(tHeader);



	switch(theader_ptr->receiveType)
	{
           
		case MODULE_TYPE_CAR_CHASSIS:
		{
			CAN1_SendBlock(CAN_ID_CAR_GIMBAL, p_buf, len);
//			if(len <= FIFO_S_CountFree(GetCan1TxFifo()))
//            {	
////                FIFO_S_Puts(GetCan1TxFifo(),p_buf,len);	
////                EventBits_t uxBits = xEventGroupSetBits(xGlobalEventGroup, EVENT_CAN1_SEND);
////				if((uxBits & EVENT_CAN1_SEND) != 0){}	
//            }
		}break;
		 
        case MODULE_TYPE_MINI_PC:
		{
			UART6_PrintBlock(p_buf, len);
		}break;
	
        case MODULE_TYPE_PC:  
		{
			UART3_PrintBlock(p_buf, len);
		}break;
		
		case ALL_MODULE:                   
		{
			UART6_PrintBlock(p_buf, len);
            CAN1_SendBlock(CAN_ID_CAR_GIMBAL, p_buf, len);			
		}break;
		
        default:	
            break;
	}

	
}

uint8_t ProtocolUnpack(tUnpackObj *pObj)
{
    uint8_t byte = 0;
    uint8_t headerLen = sizeof(tHeader);
    uint8_t unpackResult = 0;
    while( FIFO_S_CountUsed(pObj->fifoBuffer))
    {
        byte = FIFO_S_Get(pObj->fifoBuffer); 
        switch(pObj->unpackStep)
        {
            case 0:  // sof 
                if(byte == PROTOCAL_HEADER)
                {
                    pObj->unpackStep = 1;
                    pObj->protocolPacket[pObj->index++] = byte;      
                }
                else
                {
                    pObj->index = 0;
                }
                break;
            case 1:
                    pObj->dataLen = byte;
                    pObj->protocolPacket[pObj->index++] = byte;
                    pObj->unpackStep = 2;
                break;
            case 2:    // data_len high
                    pObj->dataLen |= (byte<<8);
                    pObj->protocolPacket[pObj->index++] = byte;

                    if(   pObj->dataLen  <   ( PROTOCAL_FRAME_MAX_SIZE - headerLen - sizeof(tCheckSum) ) )
                    {  
                        pObj->unpackStep = 3;	
                    }
                    else
                    {
                        pObj->unpackStep = 0;
                        pObj->index = 0;   
                    }                
                    break;

            case 3:
                    pObj->protocolPacket[pObj->index++] = byte;

                    if (pObj->index == sizeof(tHeader))
                    {
                        if(Verify_CRC8_Check_Sum(pObj->protocolPacket, headerLen) )
                        {
                            pObj->unpackStep = 4;
                        }
                        else
                        {
                            unpackResult = 1;
                            pObj->unpackStep = 0;
                            pObj->index = 0;
                        }
                    }
                    break;  

            case 4:
                if (pObj->index < (headerLen + pObj->dataLen + sizeof(tCheckSum) ) )
                {
                     pObj->protocolPacket[pObj->index++] = byte;  
                }
                if ( pObj->index >= (headerLen+ pObj->dataLen + sizeof(tCheckSum) ) )
                {
                    pObj->index = 0;
                    pObj->unpackStep = 0;    

                    if(Verify_CRC16_Check_Sum(pObj->protocolPacket, headerLen + pObj->dataLen + sizeof(tCheckSum)))
                    {										

                            if ((pObj->pHeader->receiveType == ALL_MODULE))  // all reciever
                            {																
                                ProPackageHandle(pObj->protocolPacket,headerLen + pObj->dataLen + sizeof(tCheckSum) );
                                Forward_Packet(pObj->protocolPacket,headerLen + pObj->dataLen + sizeof(tCheckSum));
                            }
                            else
                            {
                                if(pObj->pHeader->receiveType == MODULE_TYPE )
                                {
//                                    CheckModlueSeqNum(pObj->pHeader->sendType,pObj->pHeader->sendID,pObj->pHeader->seq_num);
                                    ProPackageHandle(pObj->protocolPacket,headerLen + pObj->dataLen + sizeof(tCheckSum) );
                                }
                                else
                                {
                                    Forward_Packet(pObj->protocolPacket,headerLen + pObj->dataLen + sizeof(tCheckSum));
                                }

                            }										
	
                    }
                    else
                    {
                        unpackResult = 1;
                    }
                }
                break;

            default:
                pObj->unpackStep = 0;
                pObj->index = 0;
                break;
        }
    }
    return unpackResult;
}

void InitUnpackObj(tUnpackObj *pUnpackObj, FIFO_S_t * fifoBuffer)
{
    pUnpackObj->fifoBuffer = fifoBuffer;
    pUnpackObj->index = 0;
    pUnpackObj->unpackStep = 0;
    pUnpackObj->dataLen = 0;
    pUnpackObj->pHeader = (tHeader *)pUnpackObj->protocolPacket;
}

void ProtocolInit(void)
{
	FIFO_S_Init(&Can1RxFifo,gCan1RxBuffer,CAN_RX_BUF_SIZE);      
    InitUnpackObj(&gCan1UnpackObj,&Can1RxFifo); 
	FIFO_S_Init(&Can1TxFifo,gCan1TxBuffer,CAN_TX_BUF_SIZE);
	
	FIFO_S_Init(&Usart3RxFifo,gUsart3RxBuffer,USART3_RX_BUF_SIZE);      
    InitUnpackObj(&gUsart3UnpackObj,&Usart3RxFifo); 
	
	FIFO_S_Init(&Usart6RxFifo,gUsart6RxBuffer,USART6_RX_BUF_SIZE);      
    InitUnpackObj(&gUsart6UnpackObj,&Usart6RxFifo); 
	
    ModuleCmdIdFuncTableInit();
	
	
}

void Protocol_Task(void* param)
{
	EventBits_t uxBits;
	const TickType_t xTicksToWait = 2 / portTICK_PERIOD_MS;
	
	ProtocolInit();
	
    while(1)
	{
		uxBits = xEventGroupWaitBits(xGlobalEventGroup, \
		                             EVENT_CAN1_SEND | EVENT_USART3_RECEIVE | EVENT_USART6_RECEIVE , \
		                             pdTRUE,pdTRUE,xTicksToWait);
		
		if((uxBits & EVENT_USART3_RECEIVE) == EVENT_USART3_RECEIVE)
		{
			uxBits &= ~EVENT_USART3_RECEIVE;
            ProtocolUnpack(&gUsart3UnpackObj);
		}
		
		if((uxBits & EVENT_USART6_RECEIVE) == EVENT_USART6_RECEIVE)
		{
			uxBits &= ~EVENT_USART6_RECEIVE;
            ProtocolUnpack(&gUsart6UnpackObj);
		}
		
		ProtocolUnpack(&gCan1UnpackObj);


		
	}
}


