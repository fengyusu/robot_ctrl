 #include "key.h"
 #include "stm32f4xx.h"
 #include "led.h"
 /**
  * @brief  ����Ƕ�������жϿ�����NVIC
  * @param  ��
  * @retval ��
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* �����ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

 /**
  * @brief  ���� PD7 Ϊ���жϿڣ��������ж����ȼ�
  * @param  ��
  * @retval ��
  */
void EXTI_Key_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;

	/*��������GPIO�ڵ�ʱ��*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD ,ENABLE);
  
  /* ʹ�� SYSCFG ʱ�� ��ʹ��GPIO�ⲿ�ж�ʱ����ʹ��SYSCFGʱ��*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;    /*ѡ�񰴼�������*/ 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	    		/*��������Ϊ����ģʽ*/ 
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    /*�������Ų�����Ҳ������*/
  GPIO_Init(GPIOD, &GPIO_InitStructure); /*ʹ������Ľṹ���ʼ������*/

	/* ���� EXTI �ж�Դ ��key1���� */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource7);

  /* ѡ�� EXTI �ж�Դ */
  EXTI_InitStructure.EXTI_Line = EXTI_Line7;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
  NVIC_Configuration();
}
void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line7) != RESET) //ȷ���Ƿ������EXTI Line�ж�
	{
		// LED ȡ��		
		LED_RED_TOGGLE();
		EXTI_ClearITPendingBit(EXTI_Line7);     //����жϱ�־λ
	}  
}
/*********************************************END OF FILE**********************/
