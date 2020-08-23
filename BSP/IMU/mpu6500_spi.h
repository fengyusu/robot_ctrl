#ifndef __MPU6500_SPI_H__
#define __MPU6500_SPI_H__

#include "stm32f4xx.h"


#define RCC_SPI_GPIO           RCC_AHB1Periph_GPIOF
#define GPIO_SPI               GPIOF
#define PIN_SOURCE_NSS         GPIO_PinSource6
#define PIN_SOURCE_SCK         GPIO_PinSource7
#define PIN_SOURCE_MISO        GPIO_PinSource8
#define PIN_SOURCE_MOSI        GPIO_PinSource9
#define GPIO_PIN_NSS           GPIO_Pin_6
#define GPIO_PIN_SCK           GPIO_Pin_7
#define GPIO_PIN_MISO          GPIO_Pin_8
#define GPIO_PIN_MOSI          GPIO_Pin_9

#define NSS_RESET()     GPIO_ResetBits(GPIO_SPI,GPIO_PIN_NSS)
#define NSS_SET()       GPIO_SetBits(GPIO_SPI,GPIO_PIN_NSS)

void MPU6500_SpiConfig(void);
uint8_t SPI5_ReadWriteByte(uint8_t TxData);
void SPI5_ReadData(uint8_t address, uint8_t* pdat, uint16_t dataLength);
void SPI5_WriteData(uint8_t address, uint8_t* pdat, uint16_t dataLength);

#endif
/******************************** end of mpu6500_spi.h ***********************************/

