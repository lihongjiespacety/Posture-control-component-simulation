#include <stdint.h>
#include "bsp.h"
#include "osapi_freertos.h"
#include "csp_if_can.h"
#include "can.h"
#include "driver_can.h"
#include "stm32f10x.h"
#include "misc.h"
#include "driver_iic.h"
/*******************************************************************************
* \fn          void driver_iic_init(uint8_t id,uint8_t mode,uint8_t add1,uint8_t add2,uint8_t speed)
* \brief       初始化IIC.
* \param[in]   id 1-2
* \param[in]   mode 0从机 1主机
* \param[in]   add1 从机地址1
* \param[in]   add2 从机地址2
* \param[in]   speed 0标准速度100k 1快速模式400k
* \note        . 
********************************************************************************
*/
void driver_iic_init(uint8_t id,uint8_t mode,uint8_t add1,uint8_t add2,uint8_t speed)
{
    /*使能时钟*/
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);
    if(id == 1)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
    }
    else if(id == 2)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
    }
    else{}
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    /*引脚初始化
        IIC1 
        PB7 I2C1_SDA
        PB6 I2C1_SCL
        IIC2 
        PB10 I2C2_SCL
        PB11 I2C2_SDA
    */
    if(id == 1)
    {
        /*引脚 PB6*/
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;       //PB6
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //GPIO_Mode_AF_OD
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        /*引脚 PB7*/
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;       //PB7
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //GPIO_Mode_AF_OD
        GPIO_Init(GPIOB, &GPIO_InitStructure); 
    }
    else if(id == 2)
    {   
        /*引脚 PB10*/
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;       //PB10
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //GPIO_Mode_AF_OD
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        /*引脚 PB11*/
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;       //PB11
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //GPIO_Mode_AF_OD
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    }
    else{}
    /*寄存器配置*/
    if(id == 1)
    {
        I2C1->CR1 = 0;          /*默认不响应广播地址0 无PEC校验 IIC模式*/
        I2C1->CR1 |= (1<<15);
        I2C1->CR1 &= ~(1<<15);  /*软复位*/
        I2C1->CR1 |= (1<<10);    /*接收数据后自动响应ACK*/
    
        I2C1->CR2 = 0;          /*默认不是能DMA*/
        if(mode == 0)
        {
            /*从模式使能缓冲区和事件错误中断*/
            I2C1->CR2 |= (1<<10); 
            I2C1->CR2 |= (1<<9); 
            I2C1->CR2 |= (1<<8);
            
            /*设置从地址*/
            I2C1->OAR1 = ((add1 & 0x7F)<<1);        /*7位地址模式*/
            I2C1->OAR2 = ((add2 & 0x7F)<<1) | 0x01; /*匹配两个地址*/
            
        }
        else
        {}
        /*时钟频率来源于APB1  2M-50M*/
        I2C1->CR2 |= RCC_Clocks.PCLK1_Frequency/1000000;   /*实际APB1的频率  10M的倍数才能达到400k  */
        
        I2C1->CCR = 0;  /*默认标准模式 占空比1:2*/
        if(speed==1)
        {
            I2C1->CCR |= 1<<15;
            I2C1->CCR |= 1<<14;
            
            /*快速模式占空比16:9 可以设置占空比16:9达到400k
                Thigh = 9* CCR * TPCLK1
                Tlow = 16* CCR * TPCLK1
            */
            I2C1->CCR |= (RCC_Clocks.PCLK1_Frequency/1000000);
             /*标准模式SCL允许最大上升沿300nS*/
            I2C1->TRISE = (RCC_Clocks.PCLK1_Frequency/3333333)+1;
        }
        else
        {
            /*标准模式占空比1:2
                Thigh = Tlow = CCR * TPCLK1
            */
            I2C1->CCR |= 5*(RCC_Clocks.PCLK1_Frequency/1000000);
             /*标准模式SCL允许最大上升沿100nS*/
            I2C1->TRISE = (RCC_Clocks.PCLK1_Frequency/1000000)+1;
        }
    }
    else if(id == 2)
    {
        I2C2->CR1 = 0;          /*默认不响应广播地址0 无PEC校验 IIC模式*/
        I2C2->CR1 |= (1<<15);
        I2C2->CR1 &= ~(1<<15);  /*软复位*/
        I2C2->CR1 |= (1<<10);    /*接收数据后自动响应ACK*/
    
        I2C2->CR2 = 0;          /*默认不是能DMA*/
        if(mode == 0)
        {
            /*从模式使能缓冲区和事件错误中断*/
            I2C2->CR2 |= (1<<10); 
            I2C2->CR2 |= (1<<9); 
            I2C2->CR2 |= (1<<8);
            
            /*设置从地址*/
            I2C2->OAR1 = ((add1 & 0x7F)<<1);        /*7位地址模式*/
            I2C2->OAR2 = ((add2 & 0x7F)<<1) | 0x01; /*匹配两个地址*/
            
        }
        else
        {}
        /*时钟频率来源于APB1  2M-50M*/
        I2C2->CR2 |= RCC_Clocks.PCLK1_Frequency/1000000;   /*实际APB1的频率  10M的倍数才能达到400k  */
        
        I2C2->CCR = 0;  /*默认标准模式 占空比1:2*/
        if(speed==1)
        {
            I2C2->CCR |= 1<<15;
            I2C2->CCR |= 1<<14;
            
            /*快速模式占空比16:9 可以设置占空比16:9达到400k
                Thigh = 9* CCR * TPCLK1
                Tlow = 16* CCR * TPCLK1
            */
            I2C2->CCR |= (RCC_Clocks.PCLK1_Frequency/10000000);
             /*标准模式SCL允许最大上升沿300nS*/
            I2C2->TRISE = (RCC_Clocks.PCLK1_Frequency/3333333)+1;
        }
        else
        {
            /*标准模式占空比1:2
                Thigh = Tlow = CCR * TPCLK1
            */
            I2C2->CCR |= 5*(RCC_Clocks.PCLK1_Frequency/1000000);
             /*标准模式SCL允许最大上升沿100nS*/
            I2C2->TRISE = (RCC_Clocks.PCLK1_Frequency/1000000)+1;
        }

    }
    /*中断配置*/
    if((id == 1) && (mode==0))
    {
        NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=14 ;//抢占优先级14
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //子优先级0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQ通道使能
        NVIC_Init(&NVIC_InitStructure);  
        
        NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=14 ;//抢占优先级14
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //子优先级0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQ通道使能
        NVIC_Init(&NVIC_InitStructure);
    }
    else if((id == 2) && (mode==0))
    {
        NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=14 ;//抢占优先级14
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //子优先级0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQ通道使能
        NVIC_Init(&NVIC_InitStructure);  
        
        NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=14 ;//抢占优先级14
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //子优先级0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQ通道使能
        NVIC_Init(&NVIC_InitStructure);
    }
    /*使能*/
    if(id == 1)
    {
        I2C1->CR1 |= (1<<0);    /*使能*/
    }
    else if(id == 2)
    {
        I2C2->CR2 |= (1<<0);    /*使能*/
    }
}


#if 0
/*******************************************************************************
* \fn          void I2C1_EV_IRQHandler(void)
* \brief       I2C1 事件中断处理函数.
* \note        . 
********************************************************************************
*/
void I2C1_EV_IRQHandler(void)
{


}

/*******************************************************************************
* \fn          void I2C1_ER_IRQHandler(void)
* \brief       I2C1 错误中断处理函数.
* \note        . 
********************************************************************************
*/
void I2C1_ER_IRQHandler(void)
{


}

/*******************************************************************************
* \fn          void I2C2_EV_IRQHandler(void)
* \brief       I2C2 事件中断处理函数.
* \note        . 
********************************************************************************
*/
void I2C2_EV_IRQHandler(void)
{


}

/*******************************************************************************
* \fn          void I2C2_ER_IRQHandler(void)
* \brief       I2C2 错误中断处理函数.
* \note        . 
********************************************************************************
*/
void I2C2_ER_IRQHandler(void)
{


}
#endif
/*******************************************************************************
* \fn          void driver_iic_send(uint8_t add,uint8_t reg,uint8_t* buff, uint8_t len)
* \brief       写iic.
* \param[in]   id 1-2
* \param[in]   add 从机 地址
* \param[in]   reg 寄存器地址
* \param[in]   buff 待发送数据
* \param[in]   len 待发送数据长度
* \retval      0 成功
* \retval      其他值 失败
* \note        . 
********************************************************************************
*/
int32_t driver_iic_send(uint8_t id,uint8_t add,uint8_t reg,uint8_t* buff, uint8_t len)
{
    volatile uint8_t status = 0;
    uint8_t i;
    volatile uint32_t timeout = 0;
    if(id == 1)
    {
        /*1.start转为主机模式*/
        I2C1->CR1 |= 1<<8;
        //timeout = IIC_TIMEOUT;
        //while((I2C1->CR1 & (1<<8)) && (timeout--));         /*等待发送完*/
        timeout = IIC_TIMEOUT;
        while(((I2C1->SR1 & (1<<0)) == 0) && (timeout--));  /*等待标志SB=1就绪 EV5*/
        if((I2C1->SR1 & (1<<0)) == 0)
        {
            I2C1->CR1 |= 1<<9;  
            return -1;                    /*开始位发送失败 SB未置位*/
        }
        /*2.写从设备地址*/
        I2C1->DR = add << 1;               /*EV5 SB=1 读SR1后写DR寄存器清除SB*/
        timeout = IIC_TIMEOUT;
        while(((I2C1->SR1 & (1<<2)) == 0) && (timeout--));  /*EV6 ADDR=1 等待地址发送完标志就绪ADDR=1  必须有ACK才会置位*/
        if((I2C1->SR1 & (1<<2)) == 0)
        {
            ///I2C1->CR1 |= 1<<9;  
            ///return -1;                                    /*地址字节发送失败 ADDR未置位*/
        }
        status = I2C1->SR2;                                  /*读SR1后接着读SR2 清除发送地址标志ADDR*/
        
        /*3.写寄存器地址*/
        I2C1->DR = reg;                                     /*EV8_1 移位寄存器和数据寄存器都空  TxE=1 写寄存器值 */
        timeout = IIC_TIMEOUT;
        while(((I2C1->SR1 & (1<<7)) == 0)&& (timeout--));  /*接收到ACK后 TxE(bit7)置位   等待TxE置位  EV8_1  继续写DR寄存器清除标志*/
        if((I2C1->SR1 & (1<<7)) == 0)
        {
            //I2C1->CR1 |= 1<<9;  
            ///return -1;                    /*寄存器发送失败 TXE BTF*/
        }
                    ///I2C1->CR1 |= 1<<9; 
                    return -1;
        /*4.写数据*/
        for(i=0;i<len;i++)
        {
            I2C1->DR = buff[i];                     /*写寄存器值*/
            timeout = IIC_TIMEOUT;
            while(((I2C1->SR1 & (1<<7)) == 0) && (timeout--));      /*接收到ACK后 TxE置位  等待TxE置位  EV8(移位寄存器非空 数据寄存器空)  读SR后继续写DR寄存器清除标志 停止清除标志*/
        }
        timeout = IIC_TIMEOUT;
        while(((I2C1->SR1 & (1<<2)) == 0) && (timeout--));       /*EV8-2 BTF=1 DR空*/
        /*5.stop*/
        ///if((I2C1->SR1 & (1<<2) != 0))   /*等待发送完  必须TxE or BTF置位时才能设置   stop清除标志*/
        {
            I2C1->CR1 |= 1<<9;      
            timeout = IIC_TIMEOUT;
            while((I2C1->CR1 & (1<<9)) && (timeout--));  
        }  
    }
    return 0;
}


/*******************************************************************************
* \fn          void driver_iic_read(uint8_t add,uint8_t reg,uint8_t* buff, uint8_t len)
* \brief       读iic.
* \param[in]   id 1-2
* \param[in]   add 从机 地址
* \param[in]   reg 寄存器地址
* \param[in]   buff 待发送数据
* \param[in]   len 待发送数据长度
* \retval      0 成功
* \retval      其他值 失败
* \note        . 
********************************************************************************
*/
int32_t driver_iic_read(uint8_t id,uint8_t add,uint8_t reg,uint8_t* buff, uint8_t len)
{
    volatile uint8_t status = 0;
    uint8_t i;
    volatile uint32_t timeout = 0;
    if(id == 1)
    {
        /*1.start转为主机模式*/
        I2C1->CR1 |= 1<<8;
        timeout = IIC_TIMEOUT;
        while((I2C1->CR1 & (1<<8)) && (timeout--));         /*等待发送完*/
        timeout = IIC_TIMEOUT;
        while(((I2C1->SR1 & (1<<0)) == 0) && (timeout--));  /*等待标志SB=1就绪 EV5*/
        I2C1->DR = add << 1;               /*读SR1后写DR寄存器清除SB*/
        /*2.写从设备地址*/
        timeout = IIC_TIMEOUT;
        while(((I2C1->SR1 & (1<<1)) == 0) && (timeout--));  /*等待地址发送完标志就绪ADDR=1*/
        status = I2C1->SR2;                /*读SR1后接着读SR2 清除发送地址标志ADDR EV6 EV8_1*/
        /*3.写寄存器地址*/
        I2C1->DR = reg;                    /*写寄存器值*/
        timeout = IIC_TIMEOUT;
        while(((I2C1->SR1 & (1<<7)) == 0)&& (timeout--));  /*接收到ACK后 TxE置位  等待TxE置位  EV8_1  继续写DR寄存器清除标志*/
        
        /*4.restart转为主机模式*/
        I2C1->CR1 |= 1<<8;
        timeout = IIC_TIMEOUT;
        while((I2C1->CR1 & (1<<8)) && (timeout--));         /*等待发送完*/
        timeout = IIC_TIMEOUT;
        while(((I2C1->SR1 & (1<<0)) == 0) && (timeout--));  /*等待标志SB=1就绪 EV5*/
        I2C1->DR = (add << 1) | 0x01;               /*读SR1后写DR寄存器清除SB  读*/
        
        /*4.读数据*/
        for(i=0;i<len;i++)
        {
            if((i==len-1))
            {
                I2C1->CR1 &= ~(1<<10); /*最后一字节不发送ACK*/
            }
            //I2C1->DR = buff[i];                     /*写寄存器值*/
            timeout = IIC_TIMEOUT;
            while(((I2C1->SR1 & (1<<6)) == 0) && (timeout--));      /*接收到ACK后 TxE BTF置位  等待TxE BTF置位  EV8_1  读SR后继续写DR寄存器清除标志 停止清除标志*/
            buff[i] = I2C1->DR; /*读*/
        }
        /*5.stop*/
        ///if((I2C1->SR1 & (1<<2) != 0))   /*等待发送完  必须TxE or BTF置位时才能设置   stop清除标志*/
        {
            I2C1->CR1 |= 1<<9;      
            timeout = IIC_TIMEOUT;
            while((I2C1->CR1 & (1<<9)) && (timeout--));  
        }  
        I2C1->CR1 |= (1<<10); 
    }
    return 0;
}

