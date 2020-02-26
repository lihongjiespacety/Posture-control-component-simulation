#include "bsp_time.h"
#include <stdint.h>
#include "stm32f10x.h"
#include "misc.h"
#include <stdint.h>
static bsp_time_callback s_time6_callback_pfun = 0;
static bsp_time_callback s_time7_callback_pfun = 0;

/*
frq ��ʱ��Ƶ�� hz
period�ж����� us

*/
void bsp_time6_init(uint32_t frq,uint32_t period)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
  TIM6->CR1 = 0x04;  /*ARR�Ĵ��������� д������Ч ����ģʽ  ֻ��������ж�  ʹ�ܸ����¼�*/
  //TIM6->CR2
  TIM6->DIER = 0x01;  /*ʹ�ܸ����¼��ж�*/
  //TIM6->SR
  TIM6->CNT = 0;      /*���ϼ�����  �����¼�ʱ����ARR�Ĵ�����ֵ  ���Լ���������ARR-0xFFFF*/
  TIM6->PSC = RCC_Clocks.PCLK1_Frequency/frq - 1;      /*��ʱ��Ƶ�� = fCK_PSC/(PSC[15:0] + 1)  �üĴ���ֵ���¼�����ʱ д�뵽ʵ�ʹ�����PSR�Ĵ���*/
  TIM6->ARR = period-1;      /**/
  TIM6->EGR |= 0x01;  /*�ֶ����������¼� ���¼Ĵ��� ����CR1ֻʹ��������ж� �������ﲻ���ж�*/
  
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=14 ;//��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);  
  
  TIM6->CR1 |= 0x01;  /*������ʱ��*/
  
}


/*
frq ��ʱ��Ƶ�� hz
period�ж����� us

*/
void bsp_time7_init(uint32_t frq,uint32_t period)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
  TIM7->CR1 = 0x04;  /*ARR�Ĵ��������� д������Ч ����ģʽ  ֻ��������ж�  ʹ�ܸ����¼�*/
  //TIM7->CR2
  TIM7->DIER = 0x01;  /*ʹ�ܸ����¼��ж�*/
  //TIM7->SR
  TIM7->CNT = 0;      /*���ϼ�����  �����¼�ʱ����ARR�Ĵ�����ֵ  ���Լ���������ARR-0xFFFF*/
  TIM7->PSC = RCC_Clocks.PCLK1_Frequency/frq - 1;      /*��ʱ��Ƶ�� = fCK_PSC/(PSC[15:0] + 1)  �üĴ���ֵ���¼�����ʱ д�뵽ʵ�ʹ�����PSR�Ĵ���*/
  TIM7->ARR = period-1;      /**/
  TIM7->EGR |= 0x01;  /*�ֶ����������¼� ���¼Ĵ��� ����CR1ֻʹ��������ж� �������ﲻ���ж�*/
  
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=14 ;//��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);  
  
  TIM7->CR1 |= 0x01;  /*������ʱ��*/
  
}

void bsp_time6_setcallback(bsp_time_callback callbackfun)
{
  s_time6_callback_pfun = callbackfun;

}

void bsp_time7_setcallback(bsp_time_callback callbackfun)
{
  s_time7_callback_pfun = callbackfun;

}

void TIM6_IRQHandler(void)
{
  if(TIM6->SR & 0x01)
  {
    TIM6->SR = 0x00;
    if(s_time6_callback_pfun != 0)
    {
      s_time6_callback_pfun();
    }
  }
}


//static uint8_t tog = 0;
void TIM7_IRQHandler(void)
{
  if(TIM7->SR & 0x01)
  {
    TIM7->SR = 0x00;
    if(s_time7_callback_pfun != 0)
    {
        s_time7_callback_pfun();
//      if(tog == 0)
//      {
//        tog = 1;
//      GPIOA->ODR = GPIOA->ODR | (1<<0);
//      GPIOA->ODR = GPIOA->ODR | (1<<1);
//      GPIOA->ODR = GPIOA->ODR | (1<<2);
//      GPIOA->ODR = GPIOA->ODR | (1<<3);
//      GPIOA->ODR = GPIOA->ODR | (1<<4);
//      GPIOA->ODR = GPIOA->ODR | (1<<5);
//      }
//      else
//      {
//        tog =0;
//      GPIOA->ODR = GPIOA->ODR & (~(1<<0));
//      GPIOA->ODR = GPIOA->ODR  & (~ (1<<1));
//      GPIOA->ODR = GPIOA->ODR  & (~ (1<<2));
//      GPIOA->ODR = GPIOA->ODR  & (~ (1<<3));
//      GPIOA->ODR = GPIOA->ODR  & (~ (1<<4));
//      GPIOA->ODR = GPIOA->ODR  & (~ (1<<5));
//      }
    }
  }
}