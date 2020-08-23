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
*                                             FLASH地址定义
*********************************************************************************************************
*/

//FLASH起始地址
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址
 

//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//扇区0起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//扇区1起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//扇区2起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//扇区3起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//扇区4起始地址, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//扇区5起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//扇区6起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//扇区7起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//扇区8起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//扇区9起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//扇区10起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//扇区11起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_12    ((u32)0x08100000) 	//扇区12起始地址,16 Kbytes  
#define ADDR_FLASH_SECTOR_13    ((u32)0x08104000) 	//扇区13起始地址,16 Kbytes  
#define ADDR_FLASH_SECTOR_14    ((u32)0x08108000) 	//扇区14起始地址,16 Kbytes  
#define ADDR_FLASH_SECTOR_15    ((u32)0x0810C000) 	//扇区15起始地址,16 Kbytes  
#define ADDR_FLASH_SECTOR_16    ((u32)0x08110000) 	//扇区16起始地址,64 Kbytes  
#define ADDR_FLASH_SECTOR_17    ((u32)0x08120000) 	//扇区17起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_18    ((u32)0x08140000) 	//扇区18起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_19    ((u32)0x08160000) 	//扇区19起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_20    ((u32)0x08180000) 	//扇区20起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_21    ((u32)0x081A0000) 	//扇区21起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_22    ((u32)0x081C0000) 	//扇区22起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_23    ((u32)0x081E0000) 	//扇区23起始地址,128 Kbytes  


/*
*********************************************************************************************************
*                                             接口声明
*********************************************************************************************************
*/
u8 BSP_FLASH_Write(u32 WriteAddr, u8 *pBuffer, u32 NumToWrite);		    //从指定地址开始写入指定长度的数据
void BSP_FLASH_Read(u32 ReadAddr, u8 *pBuffer, u32 NumToRead);   		//从指定地址开始读出指定长度的数据


#endif
/*********************************************** end of bsp_flash.h **************************************************/


