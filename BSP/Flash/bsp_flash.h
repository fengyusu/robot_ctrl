/*
*********************************************************************************************************
*                                     DJI BOARD SUPPORT PACKAGE
*
*                                   (c) Copyright 2015; Dji, Inc.
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                           INFANTRY_CONTROL_BOARD_V2
*
* Filename      : bsp_flash.h
* Version       : V0.10
* Programmer(s) : SC.H
*********************************************************************************************************
*/
#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H


/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include "stm32f4xx.h" 



/*
*********************************************************************************************************
*                                             FLASH��ַ����
*********************************************************************************************************
*/

//FLASH��ʼ��ַ
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH����ʼ��ַ
 

//FLASH ��������ʼ��ַ
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//����0��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//����1��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//����2��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//����3��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//����4��ʼ��ַ, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//����5��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//����6��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//����7��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//����8��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//����9��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//����10��ʼ��ַ,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//����11��ʼ��ַ,128 Kbytes  
#define ADDR_FLASH_SECTOR_12    ((u32)0x08100000) 	//����12��ʼ��ַ,16 Kbytes  
#define ADDR_FLASH_SECTOR_13    ((u32)0x08104000) 	//����13��ʼ��ַ,16 Kbytes  
#define ADDR_FLASH_SECTOR_14    ((u32)0x08108000) 	//����14��ʼ��ַ,16 Kbytes  
#define ADDR_FLASH_SECTOR_15    ((u32)0x0810C000) 	//����15��ʼ��ַ,16 Kbytes  
#define ADDR_FLASH_SECTOR_16    ((u32)0x08110000) 	//����16��ʼ��ַ,64 Kbytes  
#define ADDR_FLASH_SECTOR_17    ((u32)0x08120000) 	//����17��ʼ��ַ,128 Kbytes  
#define ADDR_FLASH_SECTOR_18    ((u32)0x08140000) 	//����18��ʼ��ַ,128 Kbytes  
#define ADDR_FLASH_SECTOR_19    ((u32)0x08160000) 	//����19��ʼ��ַ,128 Kbytes  
#define ADDR_FLASH_SECTOR_20    ((u32)0x08180000) 	//����20��ʼ��ַ,128 Kbytes  
#define ADDR_FLASH_SECTOR_21    ((u32)0x081A0000) 	//����21��ʼ��ַ,128 Kbytes  
#define ADDR_FLASH_SECTOR_22    ((u32)0x081C0000) 	//����22��ʼ��ַ,128 Kbytes  
#define ADDR_FLASH_SECTOR_23    ((u32)0x081E0000) 	//����23��ʼ��ַ,128 Kbytes  


/*
*********************************************************************************************************
*                                             �ӿ�����
*********************************************************************************************************
*/
u8 BSP_FLASH_Write(u32 WriteAddr, u8 *pBuffer, u32 NumToWrite);		    //��ָ����ַ��ʼд��ָ�����ȵ�����
void BSP_FLASH_Read(u32 ReadAddr, u8 *pBuffer, u32 NumToRead);   		//��ָ����ַ��ʼ����ָ�����ȵ�����


#endif
/*********************************************** end of bsp_flash.h **************************************************/

