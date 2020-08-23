#include "mpu6500_int.h"
#include "IMU_Task.h"

uint8_t isMPU6500_is_DRY = 0;   // mpu6500 interrupt�жϱ�־


void MPU6500_IntConfig(void)
{
    //�������ж�����
    GPIO_InitTypeDef    gpio;
    NVIC_InitTypeDef    nvic;
    EXTI_InitTypeDef    exti;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,  ENABLE);   
    
	GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &gpio);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, GPIO_PinSource1); 

    exti.EXTI_Line = EXTI_Line1;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling; //MPU6500�ж�Ϊ�͵�ƽ��Ч,�½��ش���
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    nvic.NVIC_IRQChannel = EXTI1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = MPU6500_PRIO_PRE;
    nvic.NVIC_IRQChannelSubPriority = MPU6500_PRIO_SUB;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    //�������ж�����
//    GPIO_StructInit(&gpio);
//    gpio.GPIO_Pin = GPIO_Pin_12;
//    gpio.GPIO_Mode = GPIO_Mode_IN;
//    gpio.GPIO_PuPd = GPIO_PuPd_UP;
//    gpio.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOF, &gpio);

//    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, GPIO_PinSource12); 

//    exti.EXTI_Line = EXTI_Line12;
//    exti.EXTI_Mode = EXTI_Mode_Interrupt;
//    exti.EXTI_Trigger = EXTI_Trigger_Rising; 
//    exti.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&exti);

//    nvic.NVIC_IRQChannel = EXTI15_10_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = MPU6500_PRIO_PRE;
//    nvic.NVIC_IRQChannelSubPriority = MPU6500_PRIO_SUB;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic);   
    
}


void EXTI1_IRQHandler()
{    
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    { 
        EXTI_ClearFlag(EXTI_Line1); 
        EXTI_ClearITPendingBit(EXTI_Line1);
        //��ȡԭʼ����
        isMPU6500_is_DRY = 1;   //mpu6500�жϱ�־
        GetPitchYawGxGyGz(); //��ȡ��̬����,�����Ѿ������������ʽ
        
        //�����¶Ȼ�
        //�����ϵ��MPU6500���¶ȷ�Χ        		
    }

}


/*********************************************** end of mpu6500_int.c **************************************************/

