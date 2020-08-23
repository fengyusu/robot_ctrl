#include "mpu6500_int.h"
#include "IMU_Task.h"

uint8_t isMPU6500_is_DRY = 0;   // mpu6500 interrupt中断标志


void MPU6500_IntConfig(void)
{
    //陀螺仪中断引脚
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
    exti.EXTI_Trigger = EXTI_Trigger_Falling; //MPU6500中断为低电平有效,下降沿触发
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    nvic.NVIC_IRQChannel = EXTI1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = MPU6500_PRIO_PRE;
    nvic.NVIC_IRQChannelSubPriority = MPU6500_PRIO_SUB;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    //磁力计中断引脚
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
        //读取原始数据
        isMPU6500_is_DRY = 1;   //mpu6500中断标志
        GetPitchYawGxGyGz(); //读取姿态数据,数据已经处理成连续方式
        
        //增加温度环
        //测试上电后MPU6500的温度范围        		
    }

}


/*********************************************** end of mpu6500_int.c **************************************************/

