#include <stdint.h>
#include <string.h>
#include "common.h"
#include "can.h"
#include "bsp_time.h"
#include "osapi_freertos.h"
#include "dev_magt.h"
#include "stm32f10x.h"

#define SWITCHNUM 6
#define SWITCHCHECKNUM 3
#define MAX_LEVEL_TIME 4000
#define CHECK_TIME 5000
static int32_t  gDutyRes[SWITCHNUM/2]={0};
static uint32_t gLevelTime[SWITCHNUM][2]={0};                   /*[][0] Ϊ�ߵ�ƽ [][1]Ϊ�͵�ƽ*/
static int32_t  gDuty[SWITCHNUM]={0};

static uint8_t  gState=(uint8_t)0x00;                       /*ȥ�����״̬*/
static uint8_t  gStateBuf[SWITCHNUM]={0x00,0x00,0x00,0x00}; /*����λ��ʾȥ��ʱ�� ����λ��ʾ״̬*/



MAGT_Data_t s_magt_data_at[MAGT_NUM];  /*��¼�ɼ�������*/


/**
 *******************************************************************************
 * \fn          int32_t dev_magt_gather(void)
 * \brief       ������������ɼ�.
 * \note        ��ʱ���ɼ����IO.
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
void dev_magt_gather(void)
{
/*
    ����λ״̬  0 ��ʼ 1���ȶ�  2���ȶ� 3�ߵ��� �ȴ����ȶ� 4 �͵��ߵȴ����ȶ�
    ����SWITCHCHECKNUM�α���һ��״̬����Ϊ���ȶ�״̬ 
*/
    uint8_t state=(uint8_t)0;
    uint8_t time=(uint8_t)0;
    uint8_t i;
    uint8_t flag=0;
    state = (GPIOD->IDR>>7) & 0x3F;    /*��ȡʵʱ״̬*/
    
    state = (state&0x03) | ((state&0x78)>>1);

    for(i=(uint8_t)0;i<6;i++)
    {
        switch(gStateBuf[i] & (uint8_t)0x0F)
        {
            case 0x00: /*��ʼʱֱ���Ե�һ�β�����Ϊ�ȶ�״̬*/
              if(state & (uint8_t)((uint8_t)1<<i))
              {
                  gStateBuf[i] = (gStateBuf[i] & (uint8_t)0xF0) | (uint8_t)1; /*����Ϊ���ȶ�״̬*/
                  gLevelTime[i][0]++;  /*�߱���ʱ�����*/
                  gState |= 1<<i;
              } 
              else
              {
                  gStateBuf[i] = (gStateBuf[i] & (uint8_t)0xF0) | (uint8_t)2; /*����Ϊ���ȶ�״̬*/
                  gLevelTime[i][1]++; /*�ͱ���ʱ�����*/
                  gState &= ~(1<<i);
              }
            break;
            case 1:   /*���ȶ�״̬ ������ �������ȶ�״̬       ����״̬���ı�
                        ���ȶ�״̬ ������ ����ȴ����ȶ�״̬   ����״̬���ı�
                       */
              if((state & (uint8_t)((uint8_t)1<<i)) == 0)
              {
                  gStateBuf[i] = (gStateBuf[i] & (uint8_t)0x00) | (uint8_t)3; /*�ȴ����ȶ�״̬ �����������ʱ��(����λ)*/
              } 
              gLevelTime[i][0]++;  /*״̬���ı� �߱���ʱ�����*/
            break;
            case 3:  /* �ȴ����ȶ�״̬ ������         ��Ϊ�Ƕ���       �ص����ȶ�״̬
                        �ȴ����ȶ�״̬ �����ͼ�ʱ��++ ��ʱ��>����ʱ��  ������ȶ�״̬ 
                                                      ��ʱ��<����ʱ��  �����ȴ����ȶ�״̬
                     */         
              if(state & (uint8_t)((uint8_t)1<<i))
              {
                  gStateBuf[i] = (gStateBuf[i] & (uint8_t)0xF0) | (uint8_t)1; /*�����ص����ȶ�״̬*/
                  gLevelTime[i][0]++;  /*״̬���ı� �߱���ʱ�����*/
              } 
              else
              {
                  time = (gStateBuf[i] & (uint8_t)0xF0) >>4;
                  if(time<(uint8_t)15)
                  {
                      gStateBuf[i] += (uint8_t)0x10; /*��ʱ������*/
                      time++;
                  }
                  if(time>=SWITCHCHECKNUM)
                  {
                      gStateBuf[i] = (gStateBuf[i] & (uint8_t)0x00) | (uint8_t)2; /*������ȶ�״̬ �����������ʱ��(����λ)*/
                      /*!!! �ɸߵ������� ��Ҫ����ռ�ձ� */
                      flag = 1;
                      gState &= ~(1<<i);
                  }
                  else
                  {
                      gLevelTime[i][0]++;  /*״̬���ı� �߱���ʱ�����*/
                  }
              }
            break;
            case 2:   /*���ȶ�״̬ ������ ����ȴ����ȶ�״̬   ����״̬���ı�
                        ���ȶ�״̬ ������ �������ȶ�״̬       ����״̬���ı�
                       */
              if(state & (uint8_t)((uint8_t)1<<i))
              {
                  gStateBuf[i] = (gStateBuf[i] & (uint8_t)0x00) | (uint8_t)4; /*�ȴ����ȶ�״̬ �����������ʱ��(����λ)*/
              } 
              gLevelTime[i][1]++; /*״̬����  �ͱ���ʱ�����*/
              break;
            case 4:  /* �ȴ����ȶ�״̬ ������         ��Ϊ�Ƕ���       �ص����ȶ�״̬
                        �ȴ����ȶ�״̬ �����߼�ʱ��++ ��ʱ��>����ʱ��  ������ȶ�״̬ 
                                                      ��ʱ��<����ʱ��  �����ȴ����ȶ�״̬
                     */         
              if(state & (uint8_t)((uint8_t)1<<i))
              {
                  time = (gStateBuf[i] & (uint8_t)0xF0) >>4;
                  if(time<(uint8_t)15)
                  {
                      gStateBuf[i] += (uint8_t)0x10; /*��ʱ������*/
                      time++;
                  }
                  if(time>=SWITCHCHECKNUM)
                  {
                      gStateBuf[i] = (gStateBuf[i] & (uint8_t)0x00) | (uint8_t)1; /*������ȶ�״̬ �����������ʱ��(����λ)*/
                      gLevelTime[i][0]++; /*״̬�ı䵽��  �߱���ʱ�����*/
                      
                      gState |= 1<<i;
                  }
                  else
                  {
                      gLevelTime[i][1]++; /*״̬����  �ͱ���ʱ�����*/
                  }
              } 
              else
              {
                  gStateBuf[i] = (gStateBuf[i] & (uint8_t)0xF0) | (uint8_t)2; /*�����ص����ȶ�״̬*/
                  gLevelTime[i][1]++; /*״̬����  �ͱ���ʱ�����*/
              }
            break;
        default:
          break;
        }
        
        /*�������*/
        if(gLevelTime[i][1] >= MAX_LEVEL_TIME)
        {
            gLevelTime[i][1] = 1;
            gLevelTime[i][0] = 0; 
            flag = 1;
        }
        if(gLevelTime[i][0] >= MAX_LEVEL_TIME)
        {
            gLevelTime[i][0] = 1;
            gLevelTime[i][1] = 0; 
            flag = 1;
        }
        
        
        if(flag)
        {
            /*����*/
            if(gLevelTime[i][1]==0)
            {
                if(gLevelTime[i][0] != 0)
                {
                    gDuty[i] = 100*100;
                }
                else
                {
                    gDuty[i] = 0;
                    
                }
            }
            else
            {
                gDuty[i] = gLevelTime[i][0]*10000/(gLevelTime[i][1]+gLevelTime[i][0]);
            }
            gLevelTime[i][0] = 0;  /*һ�����������¿�ʼ*/
            gLevelTime[i][1] = 0; 
        }
        else
        {
            /*û�и���*/
        }
        
        if(flag)
        {
            flag = 0;
            for(i=0;i<3;i++)
            {
                gDutyRes[i] = gDuty[2*i+1] - gDuty[2*i];
            }
        }
    }
}


/**
 *******************************************************************************
 * \fn          int32_t dev_magt_init(void)
 * \brief       ��ʼ��������������ɼ�.
 * \note        �����ǿ���ӿ�����.
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_magt_init(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    
    /*���� PD7-12*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    /*����PD13 PPS���*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    bsp_time7_init((uint32_t)1000000,(uint32_t)CHECK_TIME);  /*1M���� ��ʱ����1000uS*/
    bsp_time7_setcallback(dev_magt_gather);
    return 0;
}

/**
 *******************************************************************************
 * \fn          int32_t dev_magtdata_get(MAGT_Data_t* buff,uint8_t subtype,uint8_t swap))
 * \brief       ��ȡ������������.
 * \note        .
 * \param[in]   buff     �洢��ȡ��������.
 * \param[in]   subtype  0-n ����豸ָ������һ��.
 * \param[in]   swap     0С�� 1���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_magtdata_get(MAGT_Data_t* buff,uint8_t subtype,uint8_t swap)
{
    if((buff == 0) || (subtype>=MAGT_NUM))
    {
        return -1;
    }
    else
    {
        buff->mx = gDutyRes[0];
        buff->my = gDutyRes[1];
        buff->mz = gDutyRes[2];
        if(swap)
        {
          buffer_swap((uint8_t*)&(buff->mx),4);
          buffer_swap((uint8_t*)&(buff->my),4);
          buffer_swap((uint8_t*)&(buff->mz),4);
        }
        return 0;
    }
}


