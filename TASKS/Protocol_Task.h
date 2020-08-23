#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_
#include <stdint.h>
#include "fifo.h"
#include "GimbalCtrl_Task.h"

#define MODULE_TYPE            MODULE_TYPE_CAR_GIMBAL

#define CAN_ID_CAR_CHASSIS	   0x301
#define CAN_ID_CAR_GIMBAL 	   0x302

#define MAX_CMD_NUM            50  
#define CMD_REMOTE_DATA        0x100
#define CMD_CHASSIS_CTRL       0x101
#define CMD_GIMBAL_CTRL        0x102
#define CMD_SHOOTER_CTRL       0x103


#define CAN_RX_BUF_SIZE        256u
#define CAN_TX_BUF_SIZE        256u

#define USART3_RX_BUF_SIZE     256u

#define USART6_RX_BUF_SIZE     256u

#define EVENT_CAN1_RECEIVE    (0x01 << 1)
#define EVENT_CAN1_SEND       (0x01 << 2)

#define EVENT_USART3_RECEIVE  (0x01 << 3)
#define EVENT_USART3_SEND     (0x01 << 4)

#define EVENT_USART6_RECEIVE  (0x01 << 5)
#define EVENT_USART6_SEND     (0x01 << 6)

#define PROTOCAL_HEADER                0x5A
#define PROTOCAL_FRAME_MAX_SIZE        300 

typedef enum
{
	SPEED = 0,
	POSITION = 1,
}CtrlMode_e;

typedef __packed struct
{
	InputMode_e input_mode;
	CtrlMode_e  ctrl_mode;
	uint16_t    ctrl_cycle;    //控制信号发送周期
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}Chassis_Ctrl_t;

typedef __packed struct
{
    uint8_t  sof;
    uint16_t length;
    uint16_t seq_num;
    uint8_t  receiveType;
    uint8_t  receiveID;
    uint8_t  sendType;
    uint8_t  sendID;
    uint16_t cmd_id;
    uint8_t  crc8;
}tHeader;

typedef __packed struct
{
    uint16_t    crc16;             //CRC16 ，对整个协议Frame进行校验
}tCheckSum;

typedef enum
 {
    MODULE_TYPE_CAR_CHASSIS  = 1,    //底盘
    MODULE_TYPE_CAR_GIMBAL,          //云台
	MODULE_TYPE_MINI_PC,             //TX2
    MODULE_TYPE_PC = 0x80,
    ALL_MODULE  = 0xFF              
 }tMODULE_TYPE;
 
typedef void (*CanCmdFuncHandle)(uint8_t* buf, uint16_t len);           //protoco cmd handle function

typedef struct
{
  uint16_t        cmd_id;
  CanCmdFuncHandle handler;
} tModuleCmdFunctionList;

typedef struct
{
    FIFO_S_t    *fifoBuffer;
    tHeader     *pHeader;           
    uint16_t    dataLen;          
    uint8_t     protocolPacket[PROTOCAL_FRAME_MAX_SIZE];
    uint8_t     unpackStep;
    uint16_t    index;            
}tUnpackObj;



extern void Protocol_Task(void* param);
void Potocol_Transmit(uint8_t  send_type,    uint8_t send_id,    \
	                  uint8_t  recieve_type, uint8_t recieve_id, \
                      uint16_t cmd_id,       uint8_t* pdat,      uint16_t len);
FIFO_S_t * GetCan1RxFifo(void);
FIFO_S_t * GetCan1TxFifo(void);
FIFO_S_t * GetUsart3RxFifo(void);
FIFO_S_t * GetUsart6RxFifo(void);

#endif

