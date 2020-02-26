#include <stdint.h>
#include "bsp.h"
#include "osapi_freertos.h"
#include "stm32f10x.h"
#include "misc.h"
#include "driver_spi.h"


spi_callbacl_fp s_spi_calback=0;


/*******************************************************************************
* \fn          void driver_spi_init(uint8_t master,uint8_t mode,uint32_t speed,spi_callbacl_fp fun)
* \brief       初始化SPI.
* \param[in]   master 0从机 1主机
* \param[in]   mode 0-3
* \param[in]   speed 
* \param[in]   fun  从模式接收回调函数 
* \note        . 
********************************************************************************
*/
void driver_spi_init(uint8_t master,uint8_t mode,uint32_t speed,spi_callbacl_fp fun)
{
      uint8_t div;
      uint16_t mul=1;
      uint8_t i;
      uint8_t tmp;
      GPIO_InitTypeDef GPIO_InitStructure;
      NVIC_InitTypeDef NVIC_InitStructure;
      RCC_ClocksTypeDef RCC_Clocks;
      RCC_GetClocksFreq(&RCC_Clocks);
      /*使能时钟*/
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1|RCC_APB2Periph_AFIO,ENABLE);
      if(master)
      {
          /*引脚  PA4-NSS */
          GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; 
          GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    
          GPIO_Init(GPIOA, &GPIO_InitStructure);
          
          /*引脚  PA5-SPI1_SCK */
          GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
          GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    
          GPIO_Init(GPIOA, &GPIO_InitStructure);
          
          /*引脚  PA6-SPI1_MISO */
          GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
          GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    
          GPIO_Init(GPIOA, &GPIO_InitStructure);
          
          /*引脚  PA7-SPI1_MOSI */
          GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
          GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    
          GPIO_Init(GPIOA, &GPIO_InitStructure);
      }
      else
      {
          /*引脚  PA4-NSS */
          GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; 
          GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    
          GPIO_Init(GPIOA, &GPIO_InitStructure);
          
          /*引脚  PA5-SPI1_SCK */
          GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
          GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    
          GPIO_Init(GPIOA, &GPIO_InitStructure);
          
          /*引脚  PA6-SPI1_MISO */
          GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
          GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    
          GPIO_Init(GPIOA, &GPIO_InitStructure);
          
          /*引脚  PA7-SPI1_MOSI */
          GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
          GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    
          GPIO_Init(GPIOA, &GPIO_InitStructure);
      }

      
      GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
      
      /*寄存器配置*/
      SPI1->CR1 = 0x00;  /*高位在前 8位 硬件管理NSS */
      if(master)
      {
          SPI1->CR1 |= (1<<9) | (1<<8); /*主机模式 SSM=1软件管理 SSI=1时可输入输出数据*/
          SPI1->CR1 |= (1<<2); /* 主模式*/
      }
      else
      {
          /*从机模式  由硬件NSS引脚决定 SSM=0硬件管理  NSS拉低时可输入输出数据*/
      }
      SPI1->CR1 |= (mode & 0x03);
      
      SPI1->CR1 |= 7<<3;
      div = RCC_Clocks.PCLK2_Frequency/speed;
      for(i=0;i<=7;i++)
      {
          mul = mul << 1;
          
          if(mul>=div)
          {
              SPI1->CR1 =  (SPI1->CR1 & 0xFFC7) | (i<<3);
              break;
          
          }
      }
      
      SPI1->CR2 = 0x00;
      
      if(master)
      {
          //  SPI1->CR2 |= (1<<2); /*SS输出  模式*/
      }
      else
      {
          SPI1->CR2 |= (1<<5) | (1<<6); /*使能接收中断 错误中断*/
      }
      
      SPI1->I2SCFGR = 0x00; /*SPI模式*/
      
      NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=14 ;//抢占优先级14
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //子优先级0
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQ通道使能
      NVIC_Init(&NVIC_InitStructure); 
      
      s_spi_calback = fun;
      
      while(SPI1->SR & ((uint16_t)1<<0))
      {
          tmp = (uint8_t)SPI1->DR;  /**/
          tmp = tmp;
      }
      SPI1->CR1 |= 1<<6; /*使能SPI*/
}



/*******************************************************************************
* \fn          void driver_spi_trans(uint8_t* txbuff,uint8_t* rxbuff, uint8_t len)
* \brief       发送缓冲区同时读缓冲区.
* \param[in]   txbuff 发送缓冲区
* \param[in]   rxbuff 接收缓冲区
* \param[in]   len 发送接收长度 
* \note        . 
********************************************************************************
*/
void driver_spi_trans(uint8_t* txbuff,uint8_t* rxbuff, uint8_t len)
{
    uint32_t timeout;
    uint8_t i;
    volatile uint8_t sr;
    GPIO_ResetBits(GPIOA,GPIO_Pin_4);
    for(i=0;i<len;i++)
    {
        timeout = SPI_TIMEOUT;
        while(((SPI1->SR & (1<<1)) == 0) && (timeout--));      /*等待TXE==1*/
        
        SPI1->DR  = (uint16_t)txbuff[i];  /*必须按8位访问*/
        timeout = SPI_TIMEOUT;
        while(((SPI1->SR & (1<<1)) == 0) && (timeout--));      /*等待TXE==1*/
        
        timeout = SPI_TIMEOUT;
        while(((SPI1->SR & (1<<0)) == 0) && (timeout--));      /*等待接收*/
        rxbuff[i] = (uint8_t)SPI1->DR;
        sr = SPI1->SR;
    }
    GPIO_SetBits(GPIOA,GPIO_Pin_4);
}

/*******************************************************************************
* \fn          void SPI1_IRQHandler(void)
* \brief       SPI 中断处理函数.
* \note        . 
********************************************************************************
*/
void SPI1_IRQHandler(void)
{
    uint16_t state;
    int32_t res;
    uint8_t tmp;
    state = SPI1->SR;
    tmp = (uint8_t)SPI1->DR;
    if(state & 0x70)
    {
        /*错误处理*/
        res = s_spi_calback(&tmp);    
    }
    if(state & 0x01)
    {
        /*接收中断*/
        if(s_spi_calback)
        {
            res = s_spi_calback(&tmp);     
            if(res)
            {
                SPI1->DR = (uint16_t)tmp;
            }
        }
    
    }

}
