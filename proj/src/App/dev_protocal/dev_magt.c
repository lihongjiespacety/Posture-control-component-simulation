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
static uint32_t gLevelTime[SWITCHNUM][2]={0};                   /*[][0] 为高电平 [][1]为低电平*/
static int32_t  gDuty[SWITCHNUM]={0};

static uint8_t  gState=(uint8_t)0x00;                       /*去抖后的状态*/
static uint8_t  gStateBuf[SWITCHNUM]={0x00,0x00,0x00,0x00}; /*高四位表示去抖时间 低四位表示状态*/



MAGT_Data_t s_magt_data_at[MAGT_NUM];  /*记录采集的数据*/


/**
 *******************************************************************************
 * \fn          int32_t dev_magt_gather(void)
 * \brief       磁力矩器输出采集.
 * \note        定时器采集输出IO.
 * \retval      其他值 失败
 *******************************************************************************
 */
void dev_magt_gather(void)
{
/*
    低四位状态  0 初始 1高稳定  2低稳定 3高到低 等待低稳定 4 低到高等待高稳定
    连续SWITCHCHECKNUM次保持一个状态才认为是稳定状态 
*/
    uint8_t state=(uint8_t)0;
    uint8_t time=(uint8_t)0;
    uint8_t i;
    uint8_t flag=0;
    state = (GPIOD->IDR>>7) & 0x3F;    /*获取实时状态*/
    
    state = (state&0x03) | ((state&0x78)>>1);

    for(i=(uint8_t)0;i<6;i++)
    {
        switch(gStateBuf[i] & (uint8_t)0x0F)
        {
            case 0x00: /*初始时直接以第一次采样作为稳定状态*/
              if(state & (uint8_t)((uint8_t)1<<i))
              {
                  gStateBuf[i] = (gStateBuf[i] & (uint8_t)0xF0) | (uint8_t)1; /*设置为高稳定状态*/
                  gLevelTime[i][0]++;  /*高保持时间递增*/
                  gState |= 1<<i;
              } 
              else
              {
                  gStateBuf[i] = (gStateBuf[i] & (uint8_t)0xF0) | (uint8_t)2; /*设置为低稳定状态*/
                  gLevelTime[i][1]++; /*低保持时间递增*/
                  gState &= ~(1<<i);
              }
            break;
            case 1:   /*高稳定状态 读到高 继续高稳定状态       返回状态不改变
                        高稳定状态 读到低 进入等待低稳定状态   返回状态不改变
                       */
              if((state & (uint8_t)((uint8_t)1<<i)) == 0)
              {
                  gStateBuf[i] = (gStateBuf[i] & (uint8_t)0x00) | (uint8_t)3; /*等待低稳定状态 并清除抖动计时器(高四位)*/
              } 
              gLevelTime[i][0]++;  /*状态不改变 高保持时间递增*/
            break;
            case 3:  /* 等待低稳定状态 读到高         认为是抖动       回到高稳定状态
                        等待低稳定状态 读到低计时器++ 计时器>抖动时间  进入低稳定状态 
                                                      计时器<抖动时间  继续等待低稳定状态
                     */         
              if(state & (uint8_t)((uint8_t)1<<i))
              {
                  gStateBuf[i] = (gStateBuf[i] & (uint8_t)0xF0) | (uint8_t)1; /*抖动回到高稳定状态*/
                  gLevelTime[i][0]++;  /*状态不改变 高保持时间递增*/
              } 
              else
              {
                  time = (gStateBuf[i] & (uint8_t)0xF0) >>4;
                  if(time<(uint8_t)15)
                  {
                      gStateBuf[i] += (uint8_t)0x10; /*计时器递增*/
                      time++;
                  }
                  if(time>=SWITCHCHECKNUM)
                  {
                      gStateBuf[i] = (gStateBuf[i] & (uint8_t)0x00) | (uint8_t)2; /*进入低稳定状态 并清除抖动计时器(高四位)*/
                      /*!!! 由高到低跳变 需要计算占空比 */
                      flag = 1;
                      gState &= ~(1<<i);
                  }
                  else
                  {
                      gLevelTime[i][0]++;  /*状态不改变 高保持时间递增*/
                  }
              }
            break;
            case 2:   /*低稳定状态 读到高 进入等待高稳定状态   返回状态不改变
                        低稳定状态 读到低 继续低稳定状态       返回状态不改变
                       */
              if(state & (uint8_t)((uint8_t)1<<i))
              {
                  gStateBuf[i] = (gStateBuf[i] & (uint8_t)0x00) | (uint8_t)4; /*等待高稳定状态 并清除抖动计时器(高四位)*/
              } 
              gLevelTime[i][1]++; /*状态不变  低保持时间递增*/
              break;
            case 4:  /* 等待高稳定状态 读到低         认为是抖动       回到低稳定状态
                        等待高稳定状态 读到高计时器++ 计时器>抖动时间  进入高稳定状态 
                                                      计时器<抖动时间  继续等待高稳定状态
                     */         
              if(state & (uint8_t)((uint8_t)1<<i))
              {
                  time = (gStateBuf[i] & (uint8_t)0xF0) >>4;
                  if(time<(uint8_t)15)
                  {
                      gStateBuf[i] += (uint8_t)0x10; /*计时器递增*/
                      time++;
                  }
                  if(time>=SWITCHCHECKNUM)
                  {
                      gStateBuf[i] = (gStateBuf[i] & (uint8_t)0x00) | (uint8_t)1; /*进入高稳定状态 并清除抖动计时器(高四位)*/
                      gLevelTime[i][0]++; /*状态改变到高  高保持时间递增*/
                      
                      gState |= 1<<i;
                  }
                  else
                  {
                      gLevelTime[i][1]++; /*状态不变  低保持时间递增*/
                  }
              } 
              else
              {
                  gStateBuf[i] = (gStateBuf[i] & (uint8_t)0xF0) | (uint8_t)2; /*抖动回到低稳定状态*/
                  gLevelTime[i][1]++; /*状态不变  低保持时间递增*/
              }
            break;
        default:
          break;
        }
        
        /*避免溢出*/
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
            /*更新*/
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
            gLevelTime[i][0] = 0;  /*一个周期完重新开始*/
            gLevelTime[i][1] = 0; 
        }
        else
        {
            /*没有更新*/
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
 * \brief       初始化磁力矩器输出采集.
 * \note        处理磁强计子块数据.
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_magt_init(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    
    /*引脚 PD7-12*/
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
    
    /*引脚PD13 PPS输出*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    bsp_time7_init((uint32_t)1000000,(uint32_t)CHECK_TIME);  /*1M精度 定时周期1000uS*/
    bsp_time7_setcallback(dev_magt_gather);
    return 0;
}

/**
 *******************************************************************************
 * \fn          int32_t dev_magtdata_get(MAGT_Data_t* buff,uint8_t subtype,uint8_t swap))
 * \brief       获取磁力矩器数据.
 * \note        .
 * \param[in]   buff     存储获取到的数据.
 * \param[in]   subtype  0-n 多个设备指定是哪一个.
 * \param[in]   swap     0小端 1大端.
 * \retval      0 成功
 * \retval      其他值 失败
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


