#ifndef __LED_H__
#define __LED_H__

#include "stm32f4xx_gpio.h"

//LED端口定义
#define LED_RED_GPIO        GPIOE
#define LED_RED_PIN         GPIO_Pin_7
#define LED_RED_CLOCK       RCC_AHB1Periph_GPIOE

//LED端口定义
#define LED_GREEN_GPIO      GPIOF
#define LED_GREEN_PIN       GPIO_Pin_14
#define LED_GREEN_CLOCK     RCC_AHB1Periph_GPIOF

#define GREEN_LED_OFF()     GPIO_SetBits(LED_GREEN_GPIO,LED_GREEN_PIN)
#define GREEN_LED_ON()      GPIO_ResetBits(LED_GREEN_GPIO,LED_GREEN_PIN)
#define GREEN_LED_TOGGLE()  GPIO_ToggleBits(LED_GREEN_GPIO,LED_GREEN_PIN)

#define RED_LED_OFF()       GPIO_SetBits(LED_RED_GPIO,LED_RED_PIN)
#define RED_LED_ON()        GPIO_ResetBits(LED_RED_GPIO,LED_RED_PIN)
#define RED_LED_TOGGLE()    GPIO_ToggleBits(LED_RED_GPIO,LED_RED_PIN)

#define BOTH_LED_TOGGLE()   do{GREEN_LED_TOGGLE();\
                                RED_LED_TOGGLE();}while(0)
#define BOTH_LED_OFF()   do{GREEN_LED_OFF();\
                                RED_LED_OFF();}while(0)

#define LASER_OFF()     GPIO_ResetBits(GPIOG,GPIO_Pin_13)
#define LASER_ON()      GPIO_SetBits(GPIOG,GPIO_Pin_13)
                                
void Led_Configuration(void);//初始化	
    

#define GET_SHOOT_KEY()     GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)
#define GET_MOUSE_KEY()     GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)
void ShootKey_Configuration(void);
	
#endif
