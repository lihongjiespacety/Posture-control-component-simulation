#include <stdint.h>
#include <string.h>
#include "common.h"
#include "osapi_freertos.h"
#include "dev_ssoc.h"

//SSOC_Data_t s_ssoc_data_at[SSOC_NUM];  /*记录PC发过来的数据*/

static SSOC_Ctrl_t s_ssoc_ctrl;    

static uint8_t g_tx_vol_buff[VOL_LEN] ={0};
static uint8_t g_tx_data_buff[DATA_LEN] ={0};

/**
 *******************************************************************************
 * \fn          int32_t dev_ssoc_init(void)
* \brief       太敏设备模块初始化.
 * \note        .
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_ssoc_init(void)
{
    uint8_t i;
    memset(&s_ssoc_ctrl,0,sizeof(SSOC_Ctrl_t));
    memset(g_tx_vol_buff,0,sizeof(g_tx_vol_buff));
    memset(g_tx_data_buff,0,sizeof(g_tx_data_buff));
           
    s_ssoc_ctrl.state = SSOC_RCVCMD;
    
    g_tx_data_buff[0] = 0xFF;
    g_tx_data_buff[1] = 0x1A;
    g_tx_data_buff[2] = 0xCF;
    g_tx_data_buff[3] = 0xFC;
    g_tx_data_buff[4] = 0x1D;
    g_tx_data_buff[5] = 0x04;
    g_tx_data_buff[6] = 0x0A;
    
    g_tx_vol_buff[0] = 0xFF;
    g_tx_vol_buff[1] = 0x1A;
    g_tx_vol_buff[2] = 0xCF;
    g_tx_vol_buff[3] = 0xFC;
    g_tx_vol_buff[4] = 0x1D;
    g_tx_vol_buff[5] = 0x04;
    g_tx_vol_buff[6] = 0x0A;
    
    for(i=7;i<17;i++)
    {
        g_tx_data_buff[i] = i;
    }
    for(i=7;i<24;i++)
    {
        g_tx_vol_buff[i] = i;
    }
    /*数据更新到发送区*/
    g_tx_data_buff[16] = buffer_checksum(&g_tx_data_buff[5],11);
    g_tx_vol_buff[23] = buffer_checksum(&g_tx_vol_buff[5],18);
    return 0;
}


/**
 *******************************************************************************
 * \fn          int32_t dev_ssocdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       处理太敏子块数据.
 * \note        处理太敏子块数据.
 * \param[in]   buff     子块内容(不包括子块长度 子块类型 子块子类型).
 * \param[in]   subtype  子块子类型 一般多个同类型设备代表设备序号.
 * \param[in]   size     子块内容长度(不包括子块长度 子块类型 子块子类型).
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_ssocdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
{
    if((buff == 0) || (subtype>=SSOC_NUM))
    {
        return -1;
    }
    else
    {
        if(size == sizeof(SSOC_Data_t))
        {
            __disable_interrupt();  /*保证读三轴数据是同时更新的*/
//          s_ssoc_data_at[subtype].erfa  = buffer_get_int32(&buff[0]);
//          s_ssoc_data_at[subtype].beta  = buffer_get_int32(&buff[4]);
//          s_ssoc_data_at[subtype].sta   = buff[8];
//          s_ssoc_data_at[subtype].cell1 = buffer_get_int32(&buff[9]);
//          s_ssoc_data_at[subtype].cell2 = buffer_get_int32(&buff[13]);
//          s_ssoc_data_at[subtype].cell3 = buffer_get_int32(&buff[17]);
//          s_ssoc_data_at[subtype].cell4 = buffer_get_int32(&buff[21]);
            
            /*数据更新到发送区*/
//          buffer_set_float(&g_tx_data_buff[7],s_ssoc_data_at[subtype].erfa);
//          buffer_set_float(&g_tx_data_buff[11],s_ssoc_data_at[subtype].beta);
//          g_tx_data_buff[15] = s_ssoc_data_at[subtype].sta;
            memcpy(&g_tx_data_buff[7],&buff[0],9);
            g_tx_data_buff[16] = buffer_checksum(&g_tx_data_buff[5],11);
            
//          buffer_set_float(&g_tx_vol_buff[7],s_ssoc_data_at[subtype].cell1);
//          buffer_set_float(&g_tx_vol_buff[11],s_ssoc_data_at[subtype].cell2);
//          buffer_set_float(&g_tx_vol_buff[12],s_ssoc_data_at[subtype].cell3);
//          buffer_set_float(&g_tx_vol_buff[13],s_ssoc_data_at[subtype].cell4);
            memcpy(&g_tx_vol_buff[7],&buff[9],16);
            g_tx_vol_buff[23] = buffer_checksum(&g_tx_vol_buff[5],18);
            
            __enable_interrupt();  
            OsPrintf(OSAPI_DEBUG_INFO,"get ssocdata\r\n");
            return 0;
        }
        else
        {
            return -1;
        }
    }
}


/**
 *******************************************************************************
 * \fn          int32_t dev_ssoc_callback(uint8_t* param);
 * \brief       SPI字节接收回调函数.
 * \note        采用状态机处理.
 * \param[in]   param 指向接收到的数据或者需要回写的数据.
 * \retval      0 不需要回写
 * \retval      1 需要回写
 *******************************************************************************
 */
int32_t dev_ssoc_callback(uint8_t* param)
{
    uint8_t i;
    uint8_t* p;
    uint8_t res=0;
    
    switch(s_ssoc_ctrl.state)
    {
        case SSOC_RCVCMD:
            /*新增1个字节*/
            if(s_ssoc_ctrl.rxindex < sizeof(s_ssoc_ctrl.rxbuff)-1)
            {
                /*未溢出 直接添加到索引处*/
                s_ssoc_ctrl.rxindex++;
                s_ssoc_ctrl.rxbuff[s_ssoc_ctrl.rxindex] = *param;
            }
            else
            {
                /*大于长度  删除最开始的添加到最后*/
                for(i=0;i<CMD_LEN-1;i++)
                {
                    s_ssoc_ctrl.rxbuff[i] = s_ssoc_ctrl.rxbuff[i+1];
                }
                s_ssoc_ctrl.rxbuff[CMD_LEN-1]=  *param;
            }
            /*处理数据*/
            if(s_ssoc_ctrl.rxindex==CMD_LEN-1)
            {
                  /*有了一帧数据*/
                  p = s_ssoc_ctrl.rxbuff;
                  if((p[0]==0x1A) && (p[1]==0xCF) && (p[2]==0xFC) && (p[3]==0x1D))
                  {
                      if((p[4]==4) && (p[5]==1) && (p[6]==5))
                      {
                          /*读数据*/
                          s_ssoc_ctrl.state = SSOC_SENDDATA;
                          s_ssoc_ctrl.txlen = DATA_LEN;
                          s_ssoc_ctrl.txbuff = g_tx_data_buff;
                          s_ssoc_ctrl.txindex = 1;    /*先发送一字节*/
                          *param = g_tx_data_buff[0];
                          res = 1;
                      
                      }
                      else if(((p[4]==3) && (p[5]==1) && (p[6]==4)) || ((p[4]==1) && (p[5]==1) && (p[6]==2)))
                      {
                          /*读滤波后 和未滤波电压  一起处理返回一样的电压*/
                          s_ssoc_ctrl.state = SSOC_SENDDATA;
                          s_ssoc_ctrl.txlen = VOL_LEN;
                          s_ssoc_ctrl.txbuff = g_tx_vol_buff;
                          s_ssoc_ctrl.txindex = 1;    /*先发送一字节*/
                          *param = g_tx_vol_buff[0];
                          res = 1;
                      }
                      else
                      {
                          /*同步字对 但是内容错误 不处理继续下一一字节接收*/  
                      }
                  }
                  else
                  {
                      /*同步字错误 不处理继续下一一字节接收*/
                  }
              }
              else
              {
                  /*还未接到最下长度 不处理继续下一一字节接收*/
              }
          break;
          case SSOC_SENDDATA:
              *param = s_ssoc_ctrl.txbuff[s_ssoc_ctrl.txindex];
              if(s_ssoc_ctrl.txindex++ == s_ssoc_ctrl.txlen-1)
              {
                  s_ssoc_ctrl.rxindex = 0;
                  s_ssoc_ctrl.state = SSOC_RCVCMD;  /*本次是 最后一字节 则切换到收*/
              }
              res = 1;
         break;
    }
    return res;
}
