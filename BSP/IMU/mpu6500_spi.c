#include "mpu6500_spi.h"


/** @PinAssign NSS : PF6
  * @PinAssign SCK : PA7
  * @PinAssign MISO: PA8
  * @PinAssign MOSI: PA9
  */


//uint16_t SPI_RX_Buffer[AD_BUFFER_CH_SIZE][SENSOR_CNT] = {0};



/**
  * @brief  SPI1 Initialize
  * @param  None
  * @retval None
  * @Edit   River
  */
void MPU6500_SpiConfig(void)
{
	GPIO_InitTypeDef  gpio;
	SPI_InitTypeDef   spi;   

	/* !< Configuration the control pin in alternate mode*/
	RCC_AHB1PeriphClockCmd(RCC_SPI_GPIO,ENABLE);
//	GPIO_PinAFConfig(GPIO_SPI,PIN_SOURCE_NSS ,GPIO_AF_SPI5 );
	GPIO_PinAFConfig(GPIO_SPI,PIN_SOURCE_SCK ,GPIO_AF_SPI5 ); 
	GPIO_PinAFConfig(GPIO_SPI,PIN_SOURCE_MISO ,GPIO_AF_SPI5 ); 
	GPIO_PinAFConfig(GPIO_SPI,PIN_SOURCE_MOSI ,GPIO_AF_SPI5 ); 
    
	gpio.GPIO_Mode  = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd  = GPIO_PuPd_DOWN;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;     
	gpio.GPIO_Pin   = GPIO_PIN_SCK | GPIO_PIN_MISO | GPIO_PIN_MOSI;	
	GPIO_Init(GPIO_SPI, &gpio);
    
	/* Config the NSS Pin */
    gpio.GPIO_Pin   = GPIO_PIN_NSS;
	gpio.GPIO_Mode  = GPIO_Mode_OUT;
	gpio.GPIO_PuPd  = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_SPI, &gpio);  
    NSS_SET();
    
//	gpio.GPIO_PuPd  = GPIO_PuPd_UP;  
//	gpio.GPIO_Pin   = GPIO_PIN_NSS ;	
//	GPIO_Init(GPIO_SPI, &gpio);

	/* !< Enable the clock of spi*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5,ENABLE);

	/* !< Configuration the spi */
	spi.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;  //双线同步全双工
	spi.SPI_Mode              = SPI_Mode_Master;		
	spi.SPI_DataSize          = SPI_DataSize_8b;	            //8 bit
	spi.SPI_CPOL              = SPI_CPOL_Low;		            //0
	spi.SPI_CPHA              = SPI_CPHA_1Edge;	                //0    rising latch
	spi.SPI_NSS               = SPI_NSS_Soft;              	    //software mode
	spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;      //PCLK2=90MHz  SCK = 0.703MHz       
	spi.SPI_FirstBit          = SPI_FirstBit_MSB;	
	spi.SPI_CRCPolynomial     = 7;	
	SPI_Init(SPI5, &spi);  

	SPI_SSOutputCmd(SPI5,ENABLE);
//    SPI_TIModeCmd(SPI5,DISABLE);                                //TI mode
                         

	SPI_Cmd(SPI5, ENABLE);
}


/**
  * @brief  SPI read & write a 16 bit data
  * @param  TxData: the date will write to slave
  * @retval data read from slave
  * @Edit   River
  */
uint8_t SPI5_ReadWriteByte(uint8_t TxData)
{
    uint8_t tmp;
//    NSS_RESET();    
	while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET){;}   
	SPI_I2S_SendData(SPI5, TxData); 

	while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_RXNE) == RESET){;} 
    tmp = SPI_I2S_ReceiveData(SPI5); 
//    NSS_SET();    
    return tmp;    
}    
	
/**
* @fn       BSP_SPI5_ReadData    
* @brief    读取数据
* @param    address：   要读取的地址
*           pdat：      读取到的数据存放指针
*           dataLength  要读取的数据长度
* @retval   无
* @note     无
*/
void SPI5_ReadData(uint8_t address, uint8_t* pdat, uint16_t dataLength)
{
    NSS_RESET();
	while(SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_BSY)!=RESET){;}
	
	SPI5_ReadWriteByte(address);
	while(dataLength--)
	{
		*pdat = SPI5_ReadWriteByte(0xff);
		pdat++;
	}
    NSS_SET();
}	


/**
* @fn       BSP_SPI5_WriteData    
* @brief    写入数据
* @param    address：   要写入的地址
*           pdat：      写入数据存放指针
*           dataLength  数据长度
* @retval   无
* @note     无
*/
void SPI5_WriteData(uint8_t address, uint8_t* pdat, uint16_t dataLength)
{
    NSS_RESET();
	while(SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_BSY)!=RESET){;}
	
	SPI5_ReadWriteByte(address);
	while(dataLength--)
	{
		SPI5_ReadWriteByte(*pdat++);
	}
    NSS_SET();
}    



/**
  * @brief  SPI_DMA configuration
  * @param  buf_size￡oDMA buffer size
  * @param  dma_mode￡oDMA_Mode_Normal or DMA_Mode_Circular
  * @retval None
  * @Edit   River
  */
//void SPI_DMA_Rx_Config(uint16_t buf_size, uint32_t dma_mode)
//{
//    DMA_InitTypeDef DMA_InitStructure;
//    
//    //channel : DMA1_Channel2--SPI1_RX DMA1_Channel3--SPI1_TX;
//    
//    /* Enable DMA1 clock */
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//    
//    DMA_DeInit(DMA1_Channel2);
//    DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)SPI1_DR_ADDRESS;
//    DMA_InitStructure.DMA_MemoryBaseAddr        = (uint32_t)SPI_RX_Buffer;
//    DMA_InitStructure.DMA_DIR                   = DMA_DIR_PeripheralSRC;
//    DMA_InitStructure.DMA_BufferSize            = buf_size;
//    DMA_InitStructure.DMA_PeripheralInc         = DMA_PeripheralInc_Disable;//peripheral address not incress
//    DMA_InitStructure.DMA_MemoryInc             = DMA_MemoryInc_Enable;     //memory address incress
//    DMA_InitStructure.DMA_PeripheralDataSize    = DMA_PeripheralDataSize_HalfWord;//16 bit
//    DMA_InitStructure.DMA_MemoryDataSize        = DMA_MemoryDataSize_HalfWord;
//    DMA_InitStructure.DMA_Mode                  = dma_mode;
//    DMA_InitStructure.DMA_Priority              = DMA_Priority_VeryHigh;
//    DMA_InitStructure.DMA_M2M                   = DMA_M2M_Disable;
//    
//    DMA_Init(DMA1_Channel2,&DMA_InitStructure);
//    
//    /* Enable DMA1 Channel1 */
//    DMA_Cmd(DMA1_Channel2, ENABLE);    
//}





/**
  * @brief  Enable of disable the Rx DMA
  * @param  NewState the new state of DMA
  * @retval None   
  * @Edit   River
  */
//void SPI_DMA_Rx_Cmd(FunctionalState NewState)
//{   
//    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx ,NewState);
//}



/**
  * @brief  Get the SPI Rx buffer handler
  * @param  None
  * @retval buffer handler   
  * @Edit   River
  */
//uint16_t* Get_SPIRX_Handler(void)
//{
//    return &SPI_RX_Buffer[0][0];
//}



/**
  * @brief  Get the DAM receive buffer index
  * @param  None
  * @retval index range 0-AD_BUFFER_ALL_SIZE  
  * @Edit   River
  */
//uint16_t Get_DMA_OffsetIndex(void)
//{
//    uint16_t index = 0;
//    
//    index = AD_BUFFER_ALL_SIZE - DMA_GetCurrDataCounter(DMA1_Channel2);
//    
//    return index;
//}








/******************************** end of bsp_mpu6500_spi.c ***********************************/

