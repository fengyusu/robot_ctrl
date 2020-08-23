#include "led.h"

void Laser_Configuration(void)
{    	 
    GPIO_InitTypeDef  gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);          //使能GPIOF时钟

    //GPIOH10,H11初始化设置
    gpio.GPIO_Pin = GPIO_Pin_13;
    gpio.GPIO_Mode = GPIO_Mode_OUT;                         //普通输出模式
    gpio.GPIO_OType = GPIO_OType_PP;                        //推挽输出
    gpio.GPIO_Speed = GPIO_Speed_100MHz;                    //100MHz
    GPIO_Init(GPIOG, &gpio);                         //初始化
	
    BOTH_LED_OFF();

}

void Led_Configuration(void)
{    	 
    GPIO_InitTypeDef  gpio;

    RCC_AHB1PeriphClockCmd(LED_RED_CLOCK, ENABLE);          //使能GPIOF时钟
    RCC_AHB1PeriphClockCmd(LED_GREEN_CLOCK, ENABLE);        //使能GPIOF时钟

    //GPIOH10,H11初始化设置
    gpio.GPIO_Pin = LED_RED_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;                         //普通输出模式
    gpio.GPIO_OType = GPIO_OType_PP;                        //推挽输出
    gpio.GPIO_Speed = GPIO_Speed_100MHz;                    //100MHz
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;                      //浮空
    GPIO_Init(LED_RED_GPIO, &gpio);                         //初始化
    
    gpio.GPIO_Pin = LED_GREEN_PIN;
    GPIO_Init(LED_GREEN_GPIO, &gpio); 
	
    BOTH_LED_OFF();
    
    Laser_Configuration();

}

void ShootKey_Configuration(void)
{
    GPIO_InitTypeDef  gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);          //使能GPIOF时钟

    //GPIOH10,H11初始化设置
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_IN;                         //普通输出模式
    gpio.GPIO_Speed = GPIO_Speed_50MHz;                    //100MHz
    gpio.GPIO_PuPd = GPIO_PuPd_UP;                      //浮空
    GPIO_Init(GPIOA, &gpio); 
}
