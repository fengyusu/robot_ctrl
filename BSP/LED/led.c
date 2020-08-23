#include "led.h"

void Laser_Configuration(void)
{    	 
    GPIO_InitTypeDef  gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);          //ʹ��GPIOFʱ��

    //GPIOH10,H11��ʼ������
    gpio.GPIO_Pin = GPIO_Pin_13;
    gpio.GPIO_Mode = GPIO_Mode_OUT;                         //��ͨ���ģʽ
    gpio.GPIO_OType = GPIO_OType_PP;                        //�������
    gpio.GPIO_Speed = GPIO_Speed_100MHz;                    //100MHz
    GPIO_Init(GPIOG, &gpio);                         //��ʼ��
	
    BOTH_LED_OFF();

}

void Led_Configuration(void)
{    	 
    GPIO_InitTypeDef  gpio;

    RCC_AHB1PeriphClockCmd(LED_RED_CLOCK, ENABLE);          //ʹ��GPIOFʱ��
    RCC_AHB1PeriphClockCmd(LED_GREEN_CLOCK, ENABLE);        //ʹ��GPIOFʱ��

    //GPIOH10,H11��ʼ������
    gpio.GPIO_Pin = LED_RED_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;                         //��ͨ���ģʽ
    gpio.GPIO_OType = GPIO_OType_PP;                        //�������
    gpio.GPIO_Speed = GPIO_Speed_100MHz;                    //100MHz
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;                      //����
    GPIO_Init(LED_RED_GPIO, &gpio);                         //��ʼ��
    
    gpio.GPIO_Pin = LED_GREEN_PIN;
    GPIO_Init(LED_GREEN_GPIO, &gpio); 
	
    BOTH_LED_OFF();
    
    Laser_Configuration();

}

void ShootKey_Configuration(void)
{
    GPIO_InitTypeDef  gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);          //ʹ��GPIOFʱ��

    //GPIOH10,H11��ʼ������
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_IN;                         //��ͨ���ģʽ
    gpio.GPIO_Speed = GPIO_Speed_50MHz;                    //100MHz
    gpio.GPIO_PuPd = GPIO_PuPd_UP;                      //����
    GPIO_Init(GPIOA, &gpio); 
}
