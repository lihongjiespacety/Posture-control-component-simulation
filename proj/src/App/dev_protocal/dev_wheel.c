#include <stdint.h>
#include <string.h>
#include "common.h"
#include "osapi_freertos.h"
#include "dev_wheel.h"
#include "can.h"
#include "pc_protocal.h"
#include "driver_uart.h"
#include "obc_protocal.h"
#include "pc_protocal.h"

WHEEL_Data_t s_wheel_data_at[WHEEL_NUM]={0};     /*��¼PC������������*/
WHEEL_Data_t s_wheel_obcdata_at[WHEEL_NUM]={0};  /*��¼OBC�������ķ������� ��ʵ�ʵ����ӷ�����ʵ���ٶ�*/

static uint8_t Num_T[4] =  {0};    /*ң����ȷ֡����*/
static uint8_t SetNetNum_T[4] =  {0};   
static uint8_t SetSpeedNum_T[4] =  {0};   
static uint8_t LCmd_ID[4] =  {0};  /*���ִ��ָ����*/
static uint8_t Num_RC[4] =  {0};  /*����֡����*/
static uint8_t CAN_ID[4] =  {WHEEL_X_CANID,WHEEL_Y_CANID,WHEEL_Z_CANID,WHEEL_A_CANID};   /*ID*/

/**
 *******************************************************************************
 * \fn          int32_t dev_wheeldata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       ��������ӿ�����.
 * \note        ��������ӿ�����.
 * \param[in]   buff     �ӿ�����(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \param[in]   subtype  �ӿ������� һ����ͬ�����豸�����豸���.
 * \param[in]   size     �ӿ����ݳ���(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_wheeldata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
{
    if((buff == 0) || (subtype>=WHEEL_NUM))
    {
        return -1;
    }
    else
    {
        if(size == sizeof(WHEEL_Data_t))
        {
            __disable_interrupt();      /*��֤��������ͬʱ���µ�*/    
            s_wheel_data_at[subtype].speed = buffer_get_float(&buff[0]);
            s_wheel_data_at[subtype].toq = buffer_get_float(&buff[4]);
            s_wheel_data_at[subtype].mode = buff[8];
            __enable_interrupt(); 
            OsPrintf(OSAPI_DEBUG_INFO,"get wheeldata\r\n");
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
 * \fn          int32_t dev_wheeldata_get(WHEEL_Data_t* buff,uint8_t subtype))
 * \brief       ��ȡ��������.
 * \note        .
 * \param[in]   buff     �洢��ȡ��������.
 * \param[in]   subtype  0-n ����豸ָ������һ��.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_wheeldata_get(WHEEL_Data_t* buff,uint8_t subtype)
{
    if((buff == 0) || (subtype>=WHEEL_NUM))
    {
        return -1;
    }
    else
    {
        __disable_interrupt();
        memcpy(buff,&s_wheel_obcdata_at[subtype],sizeof(WHEEL_Data_t));
        __enable_interrupt();
        return 0;
    }
}

/**
 *******************************************************************************
 * \fn          int32_t dev_wheeltel_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       �������ң������.
 * \note        �������ң������.
 * \param[in]   buff     ����.
 * \param[in]   subtype  һ����ͬ�����豸�����豸���.
 * \param[in]   size     �������ݳ���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_wheeltel_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
{
    float speed;
    float toq;
    uint8_t mode;
    //uint32_t topcsize=0;
    //int32_t erro;
    uint8_t sendbufff[40] = {0};
    
    /*���յ�ң������ת����PC*/
  
    /*���յ�ң��������Ӧң���
     *      0x20, 0x01, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55
     */
    __disable_interrupt();      /*��֤��������ͬʱ���µ�*/    
    speed = s_wheel_data_at[subtype].speed;
    toq = s_wheel_data_at[subtype].toq;
    mode = s_wheel_data_at[subtype].mode;
    __enable_interrupt(); 
    
    /*���յ�ң������ת����PC*/

    if(size == 8)
    {
      
        if(buff[0] == 0x21)
        {
            /*�����������  ������ͨ������ת����PC*/ 
            s_wheel_obcdata_at[subtype].mode = s_wheel_data_at[subtype].mode;
            memcpy(&(s_wheel_obcdata_at[subtype].toq),&buff[1],4);    /*���������Ǵ�� ֱ�Ӵ�˷���*/
        }
        else if(buff[0] == 0x22)
        {
            /*�����ٶ�*/
            s_wheel_obcdata_at[subtype].mode = s_wheel_data_at[subtype].mode;
            memcpy(&(s_wheel_obcdata_at[subtype].speed),&buff[1],4);    /*���������Ǵ�� ֱ�Ӵ�˷���*/
            SetSpeedNum_T[subtype]++;
            ///pc_protocol_initbuffer(sendbufff, &topcsize,sizeof(sendbufff));
            ///pc_protocol_apendbuffer(sendbufff,&topcsize,sizeof(sendbufff),&s_wheel_obcdata_at[subtype],sizeof(WHEEL_Data_t),DATA_WHEEL,subtype);
            ///driver_uart_send(HAL_UART_4, sendbufff, topcsize, 10, &erro);
        }
        else if(buff[0] == 0x23)
        {
            /*���õ���*/

          
        }
        else if(buff[0] == 0x24)
        {
            /*����net����*/
            s_wheel_obcdata_at[subtype].mode = s_wheel_data_at[subtype].mode;
            __disable_interrupt();      /*��֤��������ͬʱ���µ�*/    
            memcpy(&(s_wheel_obcdata_at[subtype].toq),&buff[1],4);    
            __enable_interrupt(); 
            SetNetNum_T[subtype]++;
          
        }
        else if(buff[0] == 0xFE)
        {
            /*����485*/

          
        }
        else if(buff[0] == 0x2C)
        {
            /*����can��λ*/
          
        }
        if(buff[0] == 0xD2)
        {
            /*��ȡ�汾��Ϣ*/
            Num_T[subtype]++;
            LCmd_ID[subtype] = 0xD2;
            
            sendbufff[0] = 0x00;
            sendbufff[1] = 0x25;
            sendbufff[2] = 0x11;
            sendbufff[3] = 0x22;
            sendbufff[4] = 0x33;
            sendbufff[5] = 0x44;
            sendbufff[6] = 0;
            sendbufff[7] = 0;
            if((get_dev_state() & (1<< (DEV_NUM_WHEEL1+subtype)))== 0)
            {
              can_tx_raw_data(CAN_ID[subtype],GOM_OBC_CANID,sendbufff,8,CFP_SINGLE,1,10);
            }
          
        }
        else if(buff[0] == 0x20)
        {
            /*���յ�ң��������Ӧң���*/
            Num_T[subtype]++;
            LCmd_ID[subtype] = 0x20;
            
            sendbufff[0] = 0x00;
            sendbufff[1] = 0x25;
            sendbufff[2] = Num_T[subtype];
            sendbufff[3] = SetNetNum_T[subtype];
            sendbufff[4] = Num_RC[subtype];
            sendbufff[5] = SetSpeedNum_T[subtype];
            sendbufff[6] = mode;
            buffer_set_float(&sendbufff[16],speed);
            buffer_set_float(&sendbufff[20],toq);
            
            sendbufff[39]=buffer_checksum(sendbufff,39);
            if((get_dev_state() & (1<< (DEV_NUM_WHEEL1+subtype)))== 0)
            {
              can_tx_raw_data(CAN_ID[subtype],GOM_OBC_CANID,sendbufff,40,CFP_BEGIN,1,100);
            }
        }
        else
        {
            //Num_RC[subtype]++;
        }
    }
    else
    {
        Num_RC[subtype]++;
    } 
    return 0;
}


/**
 *******************************************************************************
 * \fn          int32_t dev_wheeltel_tlhandle(uint8_t* buff, uint8_t canid, uint8_t size)
 * \brief       �������ң���.
 * \note        �������ң���.
 * \param[in]   buff     ����. ָ�����ݰ���ȥ��ǰ���ֽڳ��Ⱥ��λ��
 * \param[in]   canid  id.
 * \param[in]   size     �������ݳ���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_wheeltel_tlhandle(uint8_t* buff, uint8_t canid, uint8_t size)
{
    switch(canid)
    {
      case WHEEL_X_CANID:
      __disable_interrupt(); 
      memcpy(&s_wheel_obcdata_at[0].rspeed ,&buff[14],4);
      __enable_interrupt(); 
      break;
      case WHEEL_Y_CANID:
      __disable_interrupt(); 
      memcpy(&s_wheel_obcdata_at[1].rspeed ,&buff[14],4);
      __enable_interrupt(); 
      break;
      case WHEEL_Z_CANID:
      __disable_interrupt(); 
      memcpy(&s_wheel_obcdata_at[2].rspeed ,&buff[14],4);
      __enable_interrupt(); 
      break;
      case WHEEL_A_CANID:
      __disable_interrupt(); 
      memcpy(&s_wheel_obcdata_at[3].rspeed ,&buff[14],4);
      __enable_interrupt(); 
      break;
    }
    return 0;
}