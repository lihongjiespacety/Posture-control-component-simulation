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
* \brief       ��ʼ��IIC.
* \param[in]   id 1-2
* \param[in]   mode 0�ӻ� 1����
* \param[in]   add1 �ӻ���ַ1
* \param[in]   add2 �ӻ���ַ2
* \param[in]   speed 0��׼�ٶ�100k 1����ģʽ400k
* \note        . 
********************************************************************************
*/
void driver_iic_init(uint8_t id,uint8_t mode,uint8_t add1,uint8_t add2,uint8_t speed)
{
    /*ʹ��ʱ��*/
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
    /*���ų�ʼ��
        IIC1 
        PB7 I2C1_SDA
        PB6 I2C1_SCL
        IIC2 
        PB10 I2C2_SCL
        PB11 I2C2_SDA
    */
    if(id == 1)
    {
        /*���� PB6*/
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;       //PB6
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //GPIO_Mode_AF_OD
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        /*���� PB7*/
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;       //PB7
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //GPIO_Mode_AF_OD
        GPIO_Init(GPIOB, &GPIO_InitStructure); 
    }
    else if(id == 2)
    {   
        /*���� PB10*/
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;       //PB10
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //GPIO_Mode_AF_OD
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        /*���� PB11*/
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;       //PB11
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //GPIO_Mode_AF_OD
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    }
    else{}
    /*�Ĵ�������*/
    if(id == 1)
    {
        I2C1->CR1 = 0;          /*Ĭ�ϲ���Ӧ�㲥��ַ0 ��PECУ�� IICģʽ*/
        I2C1->CR1 |= (1<<15);
        I2C1->CR1 &= ~(1<<15);  /*��λ*/
        I2C1->CR1 |= (1<<10);    /*�������ݺ��Զ���ӦACK*/
    
        I2C1->CR2 = 0;          /*Ĭ�ϲ�����DMA*/
        if(mode == 0)
        {
            /*��ģʽʹ�ܻ��������¼������ж�*/
            I2C1->CR2 |= (1<<10); 
            I2C1->CR2 |= (1<<9); 
            I2C1->CR2 |= (1<<8);
            
            /*���ôӵ�ַ*/
            I2C1->OAR1 = ((add1 & 0x7F)<<1);        /*7λ��ַģʽ*/
            I2C1->OAR2 = ((add2 & 0x7F)<<1) | 0x01; /*ƥ��������ַ*/
            
        }
        else
        {}
        /*ʱ��Ƶ����Դ��APB1  2M-50M*/
        I2C1->CR2 |= RCC_Clocks.PCLK1_Frequency/1000000;   /*ʵ��APB1��Ƶ��  10M�ı������ܴﵽ400k  */
        
        I2C1->CCR = 0;  /*Ĭ�ϱ�׼ģʽ ռ�ձ�1:2*/
        if(speed==1)
        {
            I2C1->CCR |= 1<<15;
            I2C1->CCR |= 1<<14;
            
            /*����ģʽռ�ձ�16:9 ��������ռ�ձ�16:9�ﵽ400k
                Thigh = 9* CCR * TPCLK1
                Tlow = 16* CCR * TPCLK1
            */
            I2C1->CCR |= (RCC_Clocks.PCLK1_Frequency/1000000);
             /*��׼ģʽSCL�������������300nS*/
            I2C1->TRISE = (RCC_Clocks.PCLK1_Frequency/3333333)+1;
        }
        else
        {
            /*��׼ģʽռ�ձ�1:2
                Thigh = Tlow = CCR * TPCLK1
            */
            I2C1->CCR |= 5*(RCC_Clocks.PCLK1_Frequency/1000000);
             /*��׼ģʽSCL�������������100nS*/
            I2C1->TRISE = (RCC_Clocks.PCLK1_Frequency/1000000)+1;
        }
    }
    else if(id == 2)
    {
        I2C2->CR1 = 0;          /*Ĭ�ϲ���Ӧ�㲥��ַ0 ��PECУ�� IICģʽ*/
        I2C2->CR1 |= (1<<15);
        I2C2->CR1 &= ~(1<<15);  /*��λ*/
        I2C2->CR1 |= (1<<10);    /*�������ݺ��Զ���ӦACK*/
    
        I2C2->CR2 = 0;          /*Ĭ�ϲ�����DMA*/
        if(mode == 0)
        {
            /*��ģʽʹ�ܻ��������¼������ж�*/
            I2C2->CR2 |= (1<<10); 
            I2C2->CR2 |= (1<<9); 
            I2C2->CR2 |= (1<<8);
            
            /*���ôӵ�ַ*/
            I2C2->OAR1 = ((add1 & 0x7F)<<1);        /*7λ��ַģʽ*/
            I2C2->OAR2 = ((add2 & 0x7F)<<1) | 0x01; /*ƥ��������ַ*/
            
        }
        else
        {}
        /*ʱ��Ƶ����Դ��APB1  2M-50M*/
        I2C2->CR2 |= RCC_Clocks.PCLK1_Frequency/1000000;   /*ʵ��APB1��Ƶ��  10M�ı������ܴﵽ400k  */
        
        I2C2->CCR = 0;  /*Ĭ�ϱ�׼ģʽ ռ�ձ�1:2*/
        if(speed==1)
        {
            I2C2->CCR |= 1<<15;
            I2C2->CCR |= 1<<14;
            
            /*����ģʽռ�ձ�16:9 ��������ռ�ձ�16:9�ﵽ400k
                Thigh = 9* CCR * TPCLK1
                Tlow = 16* CCR * TPCLK1
            */
            I2C2->CCR |= (RCC_Clocks.PCLK1_Frequency/10000000);
             /*��׼ģʽSCL�������������300nS*/
            I2C2->TRISE = (RCC_Clocks.PCLK1_Frequency/3333333)+1;
        }
        else
        {
            /*��׼ģʽռ�ձ�1:2
                Thigh = Tlow = CCR * TPCLK1
            */
            I2C2->CCR |= 5*(RCC_Clocks.PCLK1_Frequency/1000000);
             /*��׼ģʽSCL�������������100nS*/
            I2C2->TRISE = (RCC_Clocks.PCLK1_Frequency/1000000)+1;
        }

    }
    /*�ж�����*/
    if((id == 1) && (mode==0))
    {
        NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=14 ;//��ռ���ȼ�14
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //�����ȼ�0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQͨ��ʹ��
        NVIC_Init(&NVIC_InitStructure);  
        
        NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=14 ;//��ռ���ȼ�14
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //�����ȼ�0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQͨ��ʹ��
        NVIC_Init(&NVIC_InitStructure);
    }
    else if((id == 2) && (mode==0))
    {
        NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=14 ;//��ռ���ȼ�14
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //�����ȼ�0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQͨ��ʹ��
        NVIC_Init(&NVIC_InitStructure);  
        
        NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=14 ;//��ռ���ȼ�14
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //�����ȼ�0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQͨ��ʹ��
        NVIC_Init(&NVIC_InitStructure);
    }
    /*ʹ��*/
    if(id == 1)
    {
        I2C1->CR1 |= (1<<0);    /*ʹ��*/
    }
    else if(id == 2)
    {
        I2C2->CR2 |= (1<<0);    /*ʹ��*/
    }
}


#if 0
/*******************************************************************************
* \fn          void I2C1_EV_IRQHandler(void)
* \brief       I2C1 �¼��жϴ�����.
* \note        . 
********************************************************************************
*/
void I2C1_EV_IRQHandler(void)
{


}

/*******************************************************************************
* \fn          void I2C1_ER_IRQHandler(void)
* \brief       I2C1 �����жϴ�����.
* \note        . 
********************************************************************************
*/
void I2C1_ER_IRQHandler(void)
{


}

/*******************************************************************************
* \fn          void I2C2_EV_IRQHandler(void)
* \brief       I2C2 �¼��жϴ�����.
* \note        . 
********************************************************************************
*/
void I2C2_EV_IRQHandler(void)
{


}

/*******************************************************************************
* \fn          void I2C2_ER_IRQHandler(void)
* \brief       I2C2 �����жϴ�����.
* \note        . 
********************************************************************************
*/
void I2C2_ER_IRQHandler(void)
{


}
#endif
/*******************************************************************************
* \fn          void driver_iic_send(uint8_t add,uint8_t reg,uint8_t* buff, uint8_t len)
* \brief       дiic.
* \param[in]   id 1-2
* \param[in]   add �ӻ� ��ַ
* \param[in]   reg �Ĵ�����ַ
* \param[in]   buff ����������
* \param[in]   len ���������ݳ���
* \retval      0 �ɹ�
* \retval      ����ֵ ʧ��
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
        /*1.startתΪ����ģʽ*/
        I2C1->CR1 |= 1<<8;
        //timeout = IIC_TIMEOUT;
        //while((I2C1->CR1 & (1<<8)) && (timeout--));         /*�ȴ�������*/
        timeout = IIC_TIMEOUT;
        while(((I2C1->SR1 & (1<<0)) == 0) && (timeout--));  /*�ȴ���־SB=1���� EV5*/
        if((I2C1->SR1 & (1<<0)) == 0)
        {
            I2C1->CR1 |= 1<<9;  
            return -1;                    /*��ʼλ����ʧ�� SBδ��λ*/
        }
        /*2.д���豸��ַ*/
        I2C1->DR = add << 1;               /*EV5 SB=1 ��SR1��дDR�Ĵ������SB*/
        timeout = IIC_TIMEOUT;
        while(((I2C1->SR1 & (1<<2)) == 0) && (timeout--));  /*EV6 ADDR=1 �ȴ���ַ�������־����ADDR=1  ������ACK�Ż���λ*/
        if((I2C1->SR1 & (1<<2)) == 0)
        {
            ///I2C1->CR1 |= 1<<9;  
            ///return -1;                                    /*��ַ�ֽڷ���ʧ�� ADDRδ��λ*/
        }
        status = I2C1->SR2;                                  /*��SR1����Ŷ�SR2 ������͵�ַ��־ADDR*/
        
        /*3.д�Ĵ�����ַ*/
        I2C1->DR = reg;                                     /*EV8_1 ��λ�Ĵ��������ݼĴ�������  TxE=1 д�Ĵ���ֵ */
        timeout = IIC_TIMEOUT;
        while(((I2C1->SR1 & (1<<7)) == 0)&& (timeout--));  /*���յ�ACK�� TxE(bit7)��λ   �ȴ�TxE��λ  EV8_1  ����дDR�Ĵ��������־*/
        if((I2C1->SR1 & (1<<7)) == 0)
        {
            //I2C1->CR1 |= 1<<9;  
            ///return -1;                    /*�Ĵ�������ʧ�� TXE BTF*/
        }
                    ///I2C1->CR1 |= 1<<9; 
                    return -1;
        /*4.д����*/
        for(i=0;i<len;i++)
        {
            I2C1->DR = buff[i];                     /*д�Ĵ���ֵ*/
            timeout = IIC_TIMEOUT;
            while(((I2C1->SR1 & (1<<7)) == 0) && (timeout--));      /*���յ�ACK�� TxE��λ  �ȴ�TxE��λ  EV8(��λ�Ĵ����ǿ� ���ݼĴ�����)  ��SR�����дDR�Ĵ��������־ ֹͣ�����־*/
        }
        timeout = IIC_TIMEOUT;
        while(((I2C1->SR1 & (1<<2)) == 0) && (timeout--));       /*EV8-2 BTF=1 DR��*/
        /*5.stop*/
        ///if((I2C1->SR1 & (1<<2) != 0))   /*�ȴ�������  ����TxE or BTF��λʱ��������   stop�����־*/
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
* \brief       ��iic.
* \param[in]   id 1-2
* \param[in]   add �ӻ� ��ַ
* \param[in]   reg �Ĵ�����ַ
* \param[in]   buff ����������
* \param[in]   len ���������ݳ���
* \retval      0 �ɹ�
* \retval      ����ֵ ʧ��
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
        /*1.startתΪ����ģʽ*/
        I2C1->CR1 |= 1<<8;
        timeout = IIC_TIMEOUT;
        while((I2C1->CR1 & (1<<8)) && (timeout--));         /*�ȴ�������*/
        timeout = IIC_TIMEOUT;
        while(((I2C1->SR1 & (1<<0)) == 0) && (timeout--));  /*�ȴ���־SB=1���� EV5*/
        I2C1->DR = add << 1;               /*��SR1��дDR�Ĵ������SB*/
        /*2.д���豸��ַ*/
        timeout = IIC_TIMEOUT;
        while(((I2C1->SR1 & (1<<1)) == 0) && (timeout--));  /*�ȴ���ַ�������־����ADDR=1*/
        status = I2C1->SR2;                /*��SR1����Ŷ�SR2 ������͵�ַ��־ADDR EV6 EV8_1*/
        /*3.д�Ĵ�����ַ*/
        I2C1->DR = reg;                    /*д�Ĵ���ֵ*/
        timeout = IIC_TIMEOUT;
        while(((I2C1->SR1 & (1<<7)) == 0)&& (timeout--));  /*���յ�ACK�� TxE��λ  �ȴ�TxE��λ  EV8_1  ����дDR�Ĵ��������־*/
        
        /*4.restartתΪ����ģʽ*/
        I2C1->CR1 |= 1<<8;
        timeout = IIC_TIMEOUT;
        while((I2C1->CR1 & (1<<8)) && (timeout--));         /*�ȴ�������*/
        timeout = IIC_TIMEOUT;
        while(((I2C1->SR1 & (1<<0)) == 0) && (timeout--));  /*�ȴ���־SB=1���� EV5*/
        I2C1->DR = (add << 1) | 0x01;               /*��SR1��дDR�Ĵ������SB  ��*/
        
        /*4.������*/
        for(i=0;i<len;i++)
        {
            if((i==len-1))
            {
                I2C1->CR1 &= ~(1<<10); /*���һ�ֽڲ�����ACK*/
            }
            //I2C1->DR = buff[i];                     /*д�Ĵ���ֵ*/
            timeout = IIC_TIMEOUT;
            while(((I2C1->SR1 & (1<<6)) == 0) && (timeout--));      /*���յ�ACK�� TxE BTF��λ  �ȴ�TxE BTF��λ  EV8_1  ��SR�����дDR�Ĵ��������־ ֹͣ�����־*/
            buff[i] = I2C1->DR; /*��*/
        }
        /*5.stop*/
        ///if((I2C1->SR1 & (1<<2) != 0))   /*�ȴ�������  ����TxE or BTF��λʱ��������   stop�����־*/
        {
            I2C1->CR1 |= 1<<9;      
            timeout = IIC_TIMEOUT;
            while((I2C1->CR1 & (1<<9)) && (timeout--));  
        }  
        I2C1->CR1 |= (1<<10); 
    }
    return 0;
}

