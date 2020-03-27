#include <stdint.h>
#include <string.h>
#include "common.h"
#include "osapi_freertos.h"
#include "dev_ssoc.h"

//SSOC_Data_t s_ssoc_data_at[SSOC_NUM];  /*��¼PC������������*/

static SSOC_Ctrl_t s_ssoc_ctrl;    

static uint8_t g_tx_vol_buff[VOL_LEN] ={0};
static uint8_t g_tx_data_buff[DATA_LEN] ={0};

/**
 *******************************************************************************
 * \fn          int32_t dev_ssoc_init(void)
* \brief       ̫���豸ģ���ʼ��.
 * \note        .
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
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
    /*���ݸ��µ�������*/
    g_tx_data_buff[16] = buffer_checksum(&g_tx_data_buff[5],11);
    g_tx_vol_buff[23] = buffer_checksum(&g_tx_vol_buff[5],18);
    return 0;
}


/**
 *******************************************************************************
 * \fn          int32_t dev_ssocdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       ����̫���ӿ�����.
 * \note        ����̫���ӿ�����.
 * \param[in]   buff     �ӿ�����(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \param[in]   subtype  �ӿ������� һ����ͬ�����豸�����豸���.
 * \param[in]   size     �ӿ����ݳ���(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
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
            __disable_interrupt();  /*��֤������������ͬʱ���µ�*/
//          s_ssoc_data_at[subtype].erfa  = buffer_get_int32(&buff[0]);
//          s_ssoc_data_at[subtype].beta  = buffer_get_int32(&buff[4]);
//          s_ssoc_data_at[subtype].sta   = buff[8];
//          s_ssoc_data_at[subtype].cell1 = buffer_get_int32(&buff[9]);
//          s_ssoc_data_at[subtype].cell2 = buffer_get_int32(&buff[13]);
//          s_ssoc_data_at[subtype].cell3 = buffer_get_int32(&buff[17]);
//          s_ssoc_data_at[subtype].cell4 = buffer_get_int32(&buff[21]);
            
            /*���ݸ��µ�������*/
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
 * \brief       SPI�ֽڽ��ջص�����.
 * \note        ����״̬������.
 * \param[in]   param ָ����յ������ݻ�����Ҫ��д������.
 * \retval      0 ����Ҫ��д
 * \retval      1 ��Ҫ��д
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
            /*����1���ֽ�*/
            if(s_ssoc_ctrl.rxindex < sizeof(s_ssoc_ctrl.rxbuff)-1)
            {
                /*δ��� ֱ����ӵ�������*/
                s_ssoc_ctrl.rxindex++;
                s_ssoc_ctrl.rxbuff[s_ssoc_ctrl.rxindex] = *param;
            }
            else
            {
                /*���ڳ���  ɾ���ʼ����ӵ����*/
                for(i=0;i<CMD_LEN-1;i++)
                {
                    s_ssoc_ctrl.rxbuff[i] = s_ssoc_ctrl.rxbuff[i+1];
                }
                s_ssoc_ctrl.rxbuff[CMD_LEN-1]=  *param;
            }
            /*��������*/
            if(s_ssoc_ctrl.rxindex==CMD_LEN-1)
            {
                  /*����һ֡����*/
                  p = s_ssoc_ctrl.rxbuff;
                  if((p[0]==0x1A) && (p[1]==0xCF) && (p[2]==0xFC) && (p[3]==0x1D))
                  {
                      if((p[4]==4) && (p[5]==1) && (p[6]==5))
                      {
                          /*������*/
                          s_ssoc_ctrl.state = SSOC_SENDDATA;
                          s_ssoc_ctrl.txlen = DATA_LEN;
                          s_ssoc_ctrl.txbuff = g_tx_data_buff;
                          s_ssoc_ctrl.txindex = 1;    /*�ȷ���һ�ֽ�*/
                          *param = g_tx_data_buff[0];
                          res = 1;
                      
                      }
                      else if(((p[4]==3) && (p[5]==1) && (p[6]==4)) || ((p[4]==1) && (p[5]==1) && (p[6]==2)))
                      {
                          /*���˲��� ��δ�˲���ѹ  һ������һ���ĵ�ѹ*/
                          s_ssoc_ctrl.state = SSOC_SENDDATA;
                          s_ssoc_ctrl.txlen = VOL_LEN;
                          s_ssoc_ctrl.txbuff = g_tx_vol_buff;
                          s_ssoc_ctrl.txindex = 1;    /*�ȷ���һ�ֽ�*/
                          *param = g_tx_vol_buff[0];
                          res = 1;
                      }
                      else
                      {
                          /*ͬ���ֶ� �������ݴ��� �����������һһ�ֽڽ���*/  
                      }
                  }
                  else
                  {
                      /*ͬ���ִ��� �����������һһ�ֽڽ���*/
                  }
              }
              else
              {
                  /*��δ�ӵ����³��� �����������һһ�ֽڽ���*/
              }
          break;
          case SSOC_SENDDATA:
              *param = s_ssoc_ctrl.txbuff[s_ssoc_ctrl.txindex];
              if(s_ssoc_ctrl.txindex++ == s_ssoc_ctrl.txlen-1)
              {
                  s_ssoc_ctrl.rxindex = 0;
                  s_ssoc_ctrl.state = SSOC_RCVCMD;  /*������ ���һ�ֽ� ���л�����*/
              }
              res = 1;
         break;
    }
    return res;
}
