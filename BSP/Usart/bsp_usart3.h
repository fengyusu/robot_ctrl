#ifndef __USART3_H__
#define __USART3_H__

#include "stm32f4xx.h"

#define RCC_USART3_GPIO                RCC_AHB1Periph_GPIOD
#define RCC_USART3_Periph              RCC_APB1Periph_USART3
#define GPIO_USART3                    GPIOD
#define PIN_SOURCE_USART3_RX           GPIO_PinSource9
#define PIN_SOURCE_USART3_TX           GPIO_PinSource8
#define GPIO_PIN_USART3_RX             GPIO_Pin_9
#define GPIO_PIN_USART3_TX             GPIO_Pin_8
#define GPIO_USART3_AF                 GPIO_AF_USART3



void USART3_Configuration(uint32_t baud);
void UART3_PrintCh(uint8_t ch);
void UART3_PrintBlock(uint8_t* pdata, uint8_t len);

#endif

