/**
********************************************************************************        
* \brief       ������(DRIVER)CANģ��(CAN)��ؽӿ�ʵ��.
* \details     Copyright (c) 2019,spacety.
*              All rights reserved.    
* \file        driver_ds18b20.h 
* \author      lihongjie@spacety.cn
* \version     1.0 
* \date        2019��5��9��
* \note        ʹ��ǰ��ο�ע��.\n
* \since       lihongjie@spacety.cn      2019��8��23��     1.0     �½� 
* \par �޶���¼
* -  2019��8��23�� ��ʼ�汾
* \par ��Դ˵��
* - RAM:              
* - ROM:
********************************************************************************
*/

#include <stdint.h>
#include "bsp.h"
#include "osapi_freertos.h"
#include "csp_if_can.h"
#include "can.h"
#include "driver_can.h"
#include "stm32f10x.h"
#include "misc.h"
static QueueHandle_t s_can_rxqueue = 0;  /*can������Ϣ����*/
static QueueHandle_t s_can_txqueue = 0;  /*can������Ϣ����*/

/*******************************************************************************    
*                                                                              *
*                             �ڲ�����                                         *
*                                                                              *
*******************************************************************************/

static uint32_t g_driver_idlist_au32[28]={0};  /*Ĭ�Ϲ�������*/
static driver_can_status_t g_can_status_t={0};
/*******************************************************************************    
*                                                                              *
*                             �ڲ�����                                         *
*                                                                              *
*******************************************************************************/


/*******************************************************************************
* \fn          static void driver_can_ioinit(void)
* \brief       ��ʼ������.
* \note        . 
********************************************************************************
*/
static void driver_can_ioinit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
  //AFIO->MAPR = (AFIO->MAPR & (uint32_t)(~((uint32_t)3<<13))) | (uint32_t)((uint32_t)2<<13);  //PB8 PB9 CAN1
  GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
  /*ʹ��ʱ��*/

  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

  /*���� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PB9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //�����������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;//PB8
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //PA12
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PA11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  
}




/**
********************************************************************************
* \fn          static int8_t driver_can_writemailbox(uint32_t id,uint8_t ide,uint8_t rtr,uint8_t *pdata,uint8_t len)
* \brief       CANд��Ϣ����.
* \note        .
* \param[in]   id ID.
* \param[in]   ide 0:��׼֡ 1 ��չ֡
* \param[in]   rtr 0:����֡ 1 Զ��֡
* \param[in]   pdata   ��������.
* \param[out]  len ���ݳ���
* \return      uint8_t 0�ɹ� 1ʧ��.
********************************************************************************
*/
static int8_t driver_can_writemailbox(driver_can_data_t data)
{
  int8_t res = 0;
  uint8_t dlc = 0;
  if(data.len_u8>8)
  {
     dlc = (uint8_t)8;
  }
  else
  {
        dlc = data.len_u8;
  }
  if(CAN1->TSR & (uint32_t)((uint32_t)1<<26))
  {
    CAN1->sTxMailBox[0].TIR = (data.id_u32<<3)  & (uint32_t)0xFFFFFFFE;  /*����ʹ��һ��ʼ�ͷ�����һ�ε�����*/
    if(0 == (data.type_u8 & (uint8_t)((uint8_t)1<<CAN_DATA_RTR_BIT)))
    {
       CAN1->sTxMailBox[0].TIR &= (uint32_t)(~(uint32_t)((uint32_t)1<<1));
    }
    else
    {
       CAN1->sTxMailBox[0].TIR |= (uint32_t)((uint32_t)1<<1);
    }
    if(0 == (data.type_u8 & (uint8_t)((uint8_t)1<<CAN_DATA_IDE_BIT)))
    {
       CAN1->sTxMailBox[0].TIR &= (uint32_t)(~(uint32_t)((uint32_t)1<<2));
       //CAN1->sTxMailBox[0].TIR |= (uint32_t)((data.id & (uint32_t)0x7FF)<<21);
    }
    else
    {
       CAN1->sTxMailBox[0].TIR |= (uint32_t)((uint32_t)1<<2);
       //CAN1->sTxMailBox[0].TIR |= (uint32_t)((data.id & (uint32_t)0x1FFFFFFF)<<3);
    }
    /*�������*/
    CAN1->sTxMailBox[0].TDTR = (uint32_t)dlc;
    CAN1->sTxMailBox[0].TDLR = (uint32_t)data.buff_au8[0] | ((uint32_t)data.buff_au8[1]<<8) | ((uint32_t)data.buff_au8[2]<<16) | ((uint32_t)data.buff_au8[3]<<24);
    CAN1->sTxMailBox[0].TDHR = (uint32_t)data.buff_au8[4] | ((uint32_t)data.buff_au8[5]<<8) | ((uint32_t)data.buff_au8[6]<<16) | ((uint32_t)data.buff_au8[7]<<24);
    /*����*/
    CAN1->sTxMailBox[0].TIR |= (uint32_t)0x01;
        g_can_status_t.send_frames++;

  }
#if 0  /*ֻʹ��һ��FIFO���� ���ⲻ������*/
  else if(CAN1->TSR & (uint32_t)((uint32_t)1<<27))
  {
    CAN1->sTxMailBox[1].TIR = (data.id_u32<<3)  & (uint32_t)0xFFFFFFFE;  /*����ʹ��һ��ʼ�ͷ�����һ�ε�����*/
    if(0 == (data.type_u8 & (uint8_t)((uint8_t)1<<CAN_DATA_RTR_BIT)))
    {
       CAN1->sTxMailBox[1].TIR &= (uint32_t)(~(uint32_t)((uint32_t)1<<1));
    }
    else
    {
       CAN1->sTxMailBox[1].TIR |= (uint32_t)((uint32_t)1<<1);
    }
    if(0 == (data.type_u8 & (uint8_t)((uint8_t)1<<CAN_DATA_IDE_BIT)))
    {
       CAN1->sTxMailBox[1].TIR &= (uint32_t)(~(uint32_t)((uint32_t)1<<2));
       //CAN1->sTxMailBox[1].TIR |= (uint32_t)((data.id & (uint32_t)0x7FF)<<21);
    }
    else
    {
       CAN1->sTxMailBox[1].TIR |= (uint32_t)((uint32_t)1<<2);
       //CAN1->sTxMailBox[1].TIR |= (uint32_t)((data.id & (uint32_t)0x1FFFFFFF)<<3);
    }
    /*�������*/
    CAN1->sTxMailBox[1].TDTR = (uint32_t)dlc;
    CAN1->sTxMailBox[1].TDLR = (uint32_t)data.buff_au8[0] | ((uint32_t)data.buff_au8[1]<<8) | ((uint32_t)data.buff_au8[2]<<16) | ((uint32_t)data.buff_au8[3]<<24);
    CAN1->sTxMailBox[1].TDHR = (uint32_t)data.buff_au8[4] | ((uint32_t)data.buff_au8[5]<<8) | ((uint32_t)data.buff_au8[6]<<16) | ((uint32_t)data.buff_au8[7]<<24);
    /*����*/
    CAN1->sTxMailBox[1].TIR |= (uint32_t)0x01;
  }
  else if(CAN1->TSR & (uint32_t)((uint32_t)1<<28))
  {
    CAN1->sTxMailBox[2].TIR = (data.id_u32<<3)  & (uint32_t)0xFFFFFFFE;  /*����ʹ��һ��ʼ�ͷ�����һ�ε�����*/
    if(0 == (data.type_u8 & (uint8_t)((uint8_t)1<<CAN_DATA_RTR_BIT)))
    {
       CAN1->sTxMailBox[2].TIR &= (uint32_t)(~(uint32_t)((uint32_t)1<<1));
    }
    else
    {
       CAN1->sTxMailBox[2].TIR |= (uint32_t)((uint32_t)1<<1);
    }
    if(0 == (data.type_u8 & (uint8_t)((uint8_t)1<<CAN_DATA_IDE_BIT)))
    {
       CAN1->sTxMailBox[2].TIR &= (uint32_t)(~(uint32_t)((uint32_t)1<<2));
       //CAN1->sTxMailBox[2].TIR |= (uint32_t)((data.id & (uint32_t)0x7FF)<<21);
    }
    else
    {
       CAN1->sTxMailBox[2].TIR |= (uint32_t)((uint32_t)1<<2);
       //CAN1->sTxMailBox[2].TIR |= (uint32_t)((data.id & (uint32_t)0x1FFFFFFF)<<3);
    }
    /*�������*/
    CAN1->sTxMailBox[2].TDTR = (uint32_t)dlc;
    CAN1->sTxMailBox[2].TDLR = (uint32_t)data.buff_au8[0] | ((uint32_t)data.buff_au8[1]<<8) | ((uint32_t)data.buff_au8[2]<<16) | ((uint32_t)data.buff_au8[3]<<24);
    CAN1->sTxMailBox[2].TDHR = (uint32_t)data.buff_au8[4] | ((uint32_t)data.buff_au8[5]<<8) | ((uint32_t)data.buff_au8[6]<<16) | ((uint32_t)data.buff_au8[7]<<24);
    /*����*/
    CAN1->sTxMailBox[2].TIR |= (uint32_t)0x01;
        g_can_status_t.send_frames++;
  }
#endif
  else
  {
    res = 1;
  
  }
  return res;
}


/**
 *****************************************************************************
 * \fn          static int8_t is_txmail_empty(void)
 * \brief       �жϷ��������Ƿ����д����.
 * \note        . 
 * \return      1 ����д���� 0������
 *****************************************************************************
 */
static int8_t is_txmail_empty(void)
{
   if(CAN1->TSR & (uint32_t)((uint32_t)1<<26))
   {
       return (int8_t)1;
   }
#if 0
   else if(CAN1->TSR & (uint32_t)((uint32_t)1<<27))
   {
       return (int8_t)1;
   }
   else if((CAN1->TSR & (uint32_t)((uint32_t)1<<28)))
   {
       return (int8_t)1;
   }
#endif
   else
   {
       return (int8_t)0;
   }
}


void USB_LP_CAN1_RX0_IRQHandler(void)
{
   /*��������*/
    BaseType_t wake;
   driver_can_data_t tmp;
    tmp.type_u8 = (uint8_t)0;
   uint8_t len = (uint8_t)0;
    uint32_t readdata_u32;
   len = (uint8_t)(CAN1->RF0R & (uint8_t)0x03);  /*�����״̬*/
   while(len--)
   {
     if(0 != (CAN1->sFIFOMailBox[0].RIR & (uint32_t)((uint32_t)1<<1)))
     {
       tmp.type_u8 |= (uint8_t)((uint8_t)1<<CAN_DATA_RTR_BIT);
     }
     else
     {
       tmp.type_u8 &= (uint8_t)(~(uint8_t)((uint8_t)1<<CAN_DATA_RTR_BIT));
     }
     if(0 !=  (CAN1->sFIFOMailBox[0].RIR & (uint32_t)((uint32_t)1<<2)))
     {
       tmp.type_u8 |= (uint8_t)((uint8_t)1<<CAN_DATA_IDE_BIT);
     }
     else
     {
       tmp.type_u8 &= (uint8_t)(~(uint8_t)((uint8_t)1<<CAN_DATA_IDE_BIT));
     }
     tmp.id_u32= (CAN1->sFIFOMailBox[0].RIR>>3) & (uint32_t)0x1FFFFFFF;
        tmp.len_u8 = (uint8_t)(CAN1->sFIFOMailBox[0].RDTR & (uint8_t)0x0F);
        if(tmp.len_u8 > (uint8_t)8)
     {
            tmp.len_u8 = (uint8_t)8;
     }
     else
     {}
        readdata_u32 = CAN1->sFIFOMailBox[0].RDLR;
        tmp.buff_au8[0] = (uint8_t)(readdata_u32 & (uint32_t)0x00FF);
     tmp.buff_au8[1] = (uint8_t)((readdata_u32>>8) & (uint32_t)0x00FF);
     tmp.buff_au8[2] = (uint8_t)((readdata_u32>>16) & (uint32_t)0x00FF);
     tmp.buff_au8[3] = (uint8_t)((readdata_u32>>24) & (uint32_t)0x00FF);
        readdata_u32 = CAN1->sFIFOMailBox[0].RDHR;
        tmp.buff_au8[4] = (uint8_t)(readdata_u32 & (uint32_t)0x00FF);
     tmp.buff_au8[5] = (uint8_t)((readdata_u32>>8) & (uint32_t)0x00FF);
     tmp.buff_au8[6] = (uint8_t)((readdata_u32>>16) & (uint32_t)0x00FF);
     tmp.buff_au8[7] = (uint8_t)((readdata_u32>>24) & (uint32_t)0x00FF);
     CAN1->RF0R |= (uint32_t)((uint32_t)1<<5);  /*�ͷ�FIFO ��ȡ��һ��*/
        g_can_status_t.rcv_frames++;
        if(pdPASS != xQueueSendToBackFromISR(s_can_rxqueue, (BaseType_t*)(&tmp), &wake))
     {
         return;   /*����������дֱ���˳�*/
     }
   }
}

void USB_HP_CAN1_TX_IRQHandler(void)
{


}


 void CAN1_RX1_IRQHandler(void)
 {
   /*��������*/
    BaseType_t wake;
   driver_can_data_t tmp;
    tmp.type_u8 = (uint8_t)0;
   uint8_t len = (uint8_t)0;
    uint32_t readdata_u32;
   len = (uint8_t)(CAN1->RF1R & (uint8_t)0x03);  /*�����״̬*/
   while(len--)
   {
     if(0 != (CAN1->sFIFOMailBox[1].RIR & (uint32_t)((uint32_t)1<<1)))
     {
       tmp.type_u8 |= (uint8_t)((uint8_t)1<<CAN_DATA_RTR_BIT);
     }
     else
     {
       tmp.type_u8 &= (uint8_t)(~(uint8_t)((uint8_t)1<<CAN_DATA_RTR_BIT));
     }
     if(0 !=  (CAN1->sFIFOMailBox[1].RIR & (uint32_t)((uint32_t)1<<2)))
     {
       tmp.type_u8 |= (uint8_t)((uint8_t)1<<CAN_DATA_IDE_BIT);
     }
     else
     {
       tmp.type_u8 &= (uint8_t)(~(uint8_t)((uint8_t)1<<CAN_DATA_IDE_BIT));
     }
     tmp.id_u32 = (CAN1->sFIFOMailBox[1].RIR>>3) & (uint32_t)0x1FFFFFFF;
        tmp.len_u8 = (uint8_t)(CAN1->sFIFOMailBox[1].RDTR & (uint8_t)0x0F);
        if(tmp.len_u8 > (uint8_t)8)
     {
            tmp.len_u8 = (uint8_t)8;
     }
     else
     {}
        readdata_u32 = CAN1->sFIFOMailBox[1].RDLR;
        tmp.buff_au8[0] = (uint8_t)(readdata_u32 & (uint32_t)0x00FF);
     tmp.buff_au8[1] = (uint8_t)((readdata_u32>>8) & (uint32_t)0x00FF);
     tmp.buff_au8[2] = (uint8_t)((readdata_u32>>16) & (uint32_t)0x00FF);
     tmp.buff_au8[3] = (uint8_t)((readdata_u32>>24) & (uint32_t)0x00FF);
        readdata_u32 = CAN1->sFIFOMailBox[1].RDHR;
        tmp.buff_au8[4] = (uint8_t)(readdata_u32 & (uint32_t)0x00FF);
     tmp.buff_au8[5] = (uint8_t)((readdata_u32>>8) & (uint32_t)0x00FF);
     tmp.buff_au8[6] = (uint8_t)((readdata_u32>>16) & (uint32_t)0x00FF);
     tmp.buff_au8[7] = (uint8_t)((readdata_u32>>24) & (uint32_t)0x00FF);
     CAN1->RF1R |= (uint32_t)((uint32_t)1<<5);  /*�ͷ�FIFO ��ȡ��һ��*/
        g_can_status_t.rcv_frames++;
        if(pdPASS != xQueueSendToBackFromISR(s_can_rxqueue, (BaseType_t*)(&tmp), &wake))
     {
         return;   /*����������дֱ���˳�*/
     }
   }
 }

 void CAN1_SCE_IRQHandler(void)
 {
    /*���� ��������*/
   CAN1->MSR |= CAN1->MSR; /*�����־*/
   CAN1->TSR |= CAN1->TSR;
   
 }



/*******************************************************************************    
*                                                                           
*                             ����ӿں���ʵ��                                   
*                                                                            
*******************************************************************************/

/**
********************************************************************************
* \fn          void driver_can_init(uint32_t baudrate)
* \brief       ��ʼ��CAN.
* \param[in]   baudrate ������.
* \note        . 
********************************************************************************
*/
void driver_can_init(uint32_t baudrate)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  uint32_t timeout = (uint32_t)10000;
  uint32_t baudset = 0;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  
  driver_can_ioinit();
  /*�ж�����*/
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=14 ;//��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);  
   
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_SCE_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=14 ;//��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);  
  
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=14 ;//��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);  
  
  NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=14 ;//��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);    
  
  /*1.�����ʼ��ģʽ:��λ����SLEEPģʽ ->SLEEP=0 INRQ=1 �����ʼ��ģʽ*/
  CAN1->MCR &= ~(1<<1); /*SLEEP=0*/
  CAN1->MCR |=  (1<<0); /*INAR=1*/
  while((CAN1->MSR & (1<<0)) ==0);  /*�ȴ�INAK��1  ��ʼ�����*/
  /*2.����*/
  CAN1->MCR &= ~(1<<16); /*����ʱ�䴥��ģʽ*/
  CAN1->MCR &= ~(1<<7); /*����ʱ�䴥��ģʽ*/
  CAN1->MCR |=  (1<<6); /*BUS-off �Զ��ָ�*/
  CAN1->MCR |=  (1<<5); /*ʹ���Զ�����*/
  CAN1->MCR &= ~(1<<4); /*ʹ���Զ��ط�*/
  CAN1->MCR &= ~(1<<3); /*���ջ���������򸲸�*/
  CAN1->MCR &= ~(1<<2); /*�������ȼ���IDȷ��*/
  
  CAN1->MSR |= (1<<4) | (1<<3) | (1<<2); /*�����־*/
  CAN1->TSR |= (uint32_t)0x000F0F0F;/*�����־*/
  CAN1->RF0R |= (1<<4) | (1<<3); /*�����־*/
  CAN1->RF1R |= (1<<4) | (1<<3); /*�����־*/
  
  
  CAN1->IER = 0x00038F7E; /*ʹ�ܳ���������������ж������ж�*/
  
  CAN1->ESR = (uint32_t)0x70;
  
  /*
  BaudRate= 1/NominalBitTime
  NominalBitTime =1xtq + tBS1 + tBS2
  tBS1 = tq x (TS1[3:0] + 1)
  tBS2 = tq x (TS2[2:0] + 1)
  tq = (BRP[9:0] + 1) x tPCLK
   ts1 ts2����Ϊ0
  BaudRate = 1/3tq
  */
  baudset = RCC_Clocks.PCLK1_Frequency/((uint32_t)8 * baudrate) -1;
  //CAN1->BTR = (uint32_t)0xC1230000 | (uint32_t)baudset;  /*�ػ�ģʽ  ���ò�����*/
  CAN1->BTR = (uint32_t)0x01230000 | (uint32_t)baudset;  /*����ģʽ  ���ò�����*/

  /*4.��������ģʽ:INRQ=0*/
  CAN1->MCR &=  ~(1<<0); /*INRQ=0*/
  while((CAN1->MSR & (1<<0)) && (timeout--));  /*�ȴ�INAK��0  �������*/
  
  /*3.���ý��չ���*/
  CAN1->FMR |= (uint32_t)0x01;         /* FINIT=1�������ú����Ĵ���*/
  CAN1->FM1R = (uint32_t)0x00000000;  /*���й�����ʹ��MASKģʽ*/
  CAN1->FS1R = (uint32_t)0x0FFFFFFF;  /*���й�����ʹ��32λƥ��ģʽ*/
  CAN1->FFA1R = (uint32_t)0x0FFFFFFF;  /*����һ��FIFO�������ʱ������*/
  CAN1->FA1R = 0;  /*�رղ�������*/
  /*����ID����*/
  CAN1->sFilterRegister[0].FR1 = (g_driver_idlist_au32[0]<<3) | (uint32_t)0x04;  /*��չ����֡0x04 ��׼����֡0x00  ��չԶ��֡0x03 ��׼Զ��֡0x02  ��׼֡<<21λ*/
  CAN1->sFilterRegister[0].FR2 = (g_driver_idlist_au32[1]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[1].FR1 = (g_driver_idlist_au32[2]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[1].FR2 = (g_driver_idlist_au32[3]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[2].FR1 = (g_driver_idlist_au32[4]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[2].FR2 = (g_driver_idlist_au32[5]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[3].FR1 = (g_driver_idlist_au32[6]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[3].FR2 = (g_driver_idlist_au32[7]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[4].FR1 = (g_driver_idlist_au32[8]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[4].FR2 = (g_driver_idlist_au32[9]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[5].FR1 = (g_driver_idlist_au32[10]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[5].FR2 = (g_driver_idlist_au32[11]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[6].FR1 = (g_driver_idlist_au32[12]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[6].FR2 = (g_driver_idlist_au32[13]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[7].FR1 = (g_driver_idlist_au32[14]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[7].FR2 = (g_driver_idlist_au32[15]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[8].FR1 = (g_driver_idlist_au32[16]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[8].FR2 = (g_driver_idlist_au32[17]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[9].FR1 = (g_driver_idlist_au32[18]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[9].FR2 = (g_driver_idlist_au32[19]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[10].FR1 = (g_driver_idlist_au32[20]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[10].FR2 = (g_driver_idlist_au32[21]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[11].FR1 = (g_driver_idlist_au32[22]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[11].FR2 = (g_driver_idlist_au32[23]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[12].FR1 = (g_driver_idlist_au32[24]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[12].FR2 = (g_driver_idlist_au32[25]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[13].FR1 = (g_driver_idlist_au32[26]<<3) | (uint32_t)0x04;
  CAN1->sFilterRegister[13].FR2 = (g_driver_idlist_au32[27]<<3) | (uint32_t)0x04;
  //CAN1->FA1R = (uint32_t)0xFFFFFFF;  /*��ʹ��*/
  CAN1->FMR &= (uint32_t)~0x01;         /* ʹ�ܹ���*/
    
    if(0 == s_can_rxqueue)
    {
        s_can_rxqueue = xQueueCreate(CAN_RXQUEUE_LEN, sizeof(driver_can_data_t));
    }
    if(0 == s_can_rxqueue)
    {
       // printf("create can_rxqueue err\r\n");
    }
    if(0 == s_can_txqueue)
    {
        s_can_txqueue = xQueueCreate(CAN_TXQUEUE_LEN, sizeof(driver_can_data_t));
    }
    if(0 == s_can_txqueue)
    {
        //printf("create can_txqueue err\r\n");
    }
    
    g_can_status_t.rcv_err = 0;
    g_can_status_t.send_err = 0;
    g_can_status_t.rcv_frames = 0;
    g_can_status_t.send_frames = 0;
    g_can_status_t.esr = 0;
}



/**
********************************************************************************
* \fn          void driver_can_filter(uint_t mode,uint32_t id,uint32_t mask)
* \brief       ����CAN����.
* \param[in]   ch ͨ��0-13.
* \param[in]   mode 0:MASKģʽ 1:listģʽ.
* \param[in]   id id.
* \param[in]   mask ����.
* \note        . 
********************************************************************************
*/
void driver_can_filter(uint8_t ch,uint8_t mode,uint32_t id,uint32_t mask)
{
    /*���ý��չ���*/
  CAN1->FMR |= (uint32_t)0x01;         /* FINIT=1�������ú����Ĵ���*/
  if(mode == (uint8_t)1)
  {
    CAN1->FM1R |= (uint32_t)((uint32_t)1<<ch);  /*ʹ��LISTģʽ*/
  }
  else
  {
    CAN1->FM1R &= (uint32_t)(~(uint32_t)((uint32_t)1<<ch));  /*ʹ��MSKģʽ*/
  }

  CAN1->FA1R &= (uint32_t)(~(uint32_t)((uint32_t)1<<ch));  /*�رղ�������*/
  /*����ID����*/
  CAN1->sFilterRegister[ch].FR1 = id<<3;  /*��չ����֡0x04 ��׼����֡0x00  ��չԶ��֡0x03 ��׼Զ��֡0x02  ��׼֡<<21λ*/
  CAN1->sFilterRegister[ch].FR2 = mask<<3;
  CAN1->FA1R |= (uint32_t)((uint32_t)1<<ch);   /*ʹ��*/
  CAN1->FMR &= (uint32_t)~0x01;         /* ʹ�ܹ���*/
}

/**
 *****************************************************************************
 * \fn          int8_t driver_can_send(driver_can_data_t data, uint32_t timeout)
 * \brief       ����CAN����(д�뻺����).
 * \note        Ӧ�õ��øú�����д���ݵ����ͻ�����,���������ȴ�. 
 * \param[in]   data ָ��can_data�ṹ��ʵ����ָ�� \ref driver_can_data_t
 * \param[in]   timeout �趨��ʱʱ�� 0��ʾһֱ�ȴ�
 * \retval      0 д�ɹ�
 * \retval      1 дʧ��
 *****************************************************************************
 */
int8_t driver_can_send(driver_can_data_t data, uint32_t timeout)
{
    if(pdPASS == xQueueSendToBack(s_can_txqueue, (BaseType_t*)(&data),timeout))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/**
 *****************************************************************************
 * \fn          int8_t driver_can_recv(driver_can_data_t* data, uint32_t timeout)
 * \brief       �ӽ��ջ�����������.
 * \note        Ӧ�ú������øú����ӽ��ջ�����������,����ָ����ʱʱ��. 
 * \param[out]  data ָ��can_data�ṹ��ʵ����ָ�� \ref driver_can_data_t
 * \param[in]   timeout ָ����ʱʱ�� 0��ʾһֱ�ȴ� 
 * \retval      int8_t 0�ɹ� ����ֵʧ��.
 *****************************************************************************
 */
int8_t driver_can_recv(driver_can_data_t* data, uint32_t timeout)
{
    if(pdPASS == xQueueReceive(s_can_rxqueue, data, timeout))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/**
 *****************************************************************************
 * \fn          int8_t driver_can_issenddown(void)
 * \brief       �鿴�������Ƿ��.
 * \note        . 
* \retval       0 �������ǿ�
* \retval       1 ��������
 *****************************************************************************
 */
int8_t driver_can_issenddown(void)
{
    driver_can_data_t data;
    if(pdPASS == xQueuePeek(s_can_txqueue,(BaseType_t*)(&data),0))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/**
 *****************************************************************************
 * \fn          int8_t driver_can_sendloop(void)
 * \brief       ���������Ϣ�����п�,���з�������,�򽫷��ͻ���������д�뷢����Ϣ����.
 * \note        Ӧ�����ڵ��øú������з��ʹ���. 
 * \retval      0 �з���
 * \retval      1 �޷���
 *****************************************************************************
 */
int8_t driver_can_sendloop(void)
{
    static uint32_t g_check_empty_u32 = 0;
    int8_t res = (int8_t)0;
    driver_can_data_t tmp;
    do
    {
        if(is_txmail_empty() != (uint8_t)0)
        {
            g_check_empty_u32 = 0;
            if(xQueueReceive(s_can_txqueue, &tmp, 5) == pdPASS)
            {
              res = driver_can_writemailbox(tmp);
            }
            else
            {
              res = (int8_t)1;
            }
        }
        else
        {
            g_check_empty_u32++;
            if(g_check_empty_u32 >= 100)  /*���100�β�ѯ����������ǿ�˵���쳣���³�ʼ��(���䷢��һ֡�����ܳ���100ms)*/
            {
                g_check_empty_u32 = 0;
                res = (int8_t)1;
                //driver_can_init((uint32_t)500000);
                //driver_can_filter(0,0,(uint32_t)can_getid()<<13,(uint32_t)0xFF<<13);
                //driver_can_filter(1,0,(uint32_t)can_getid()<<19,(uint32_t)0x1F<<19);
            }
            OsTimeDelay(1);
        }
    }
    while(res == 0);
    return res;
}

/**
 *****************************************************************************
 * \fn          int8_t driver_can_getstatus(driver_can_status_t* status_t)
 * \brief       ��ȡCAN״̬.
 * \note        . 
 * \param[in]   status_t \ref driver_can_status_t
 * \retval      0 �з���
 * \retval      1 �޷���
 *****************************************************************************
 */
int8_t driver_can_getstatus(driver_can_status_t* status_t)
{
    status_t->rcv_err = (CAN1->ESR >> 24);
    status_t->rcv_frames = g_can_status_t.rcv_frames;
    status_t->send_err = (CAN1->ESR >> 16) & 0xFF;
    status_t->send_frames = g_can_status_t.send_frames;
    status_t->esr =g_can_status_t.esr;
    return 0;
}
