#include <stdint.h>
#include <string.h>
#include <time.h>
#include "common.h"
#include "osapi_freertos.h"
#include "dev_gyro.h"
#include "can.h"
#include "clock.h"
#include "obc_protocal.h"
#include "pc_protocal.h"

GYRO_Data_t s_gyro_data_at[GYRO_NUM];  /*��¼PC������������*/
static uint8_t Num_C = 0;    /*��ȷ֡����*/
static uint8_t LCmd_ID = 0;  /*�̶�Ϊ0*/
static uint8_t St_Cmd = 0;   /*ʱ��ָ��ִ��״̬*/
static uint8_t Num_RC = 0;  /*����֡����*/

/**
 *******************************************************************************
 * \fn          int32_t dev_gyrodata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       ���������ӿ�����.
 * \note        ���������ӿ�����.
 * \param[in]   buff     �ӿ�����(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \param[in]   subtype  �ӿ������� һ����ͬ�����豸�����豸���.
 * \param[in]   size     �ӿ����ݳ���(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_gyrodata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
{
    if((buff == 0) || (subtype>=GYRO_NUM))
    {
        return -1;
    }
    else
    {
        if(size == sizeof(GYRO_Data_t))
        {
            __disable_interrupt();      /*��֤������������ͬʱ���µ�*/    
            s_gyro_data_at[subtype].x_speed  = buffer_get_int32(&buff[0]);
            s_gyro_data_at[subtype].y_speed  = buffer_get_int32(&buff[4]);
            s_gyro_data_at[subtype].z_speed  = buffer_get_int32(&buff[8]);
            s_gyro_data_at[subtype].x_temp   = buffer_get_int16(&buff[12]);
            s_gyro_data_at[subtype].y_temp   = buffer_get_int16(&buff[14]);
            s_gyro_data_at[subtype].z_temp   = buffer_get_int16(&buff[16]);
            __enable_interrupt(); 
            OsPrintf(OSAPI_DEBUG_INFO,"get gyrodata\r\n");
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
 * \fn          int32_t dev_gyrodata_get(GYRO_Data_t* buff,uint8_t subtype))
 * \brief       ��ȡ��������.
 * \note        .
 * \param[in]   buff     �洢��ȡ��������.
 * \param[in]   subtype  0-n ����豸ָ������һ��.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_gyrodata_get(GYRO_Data_t* buff,uint8_t subtype)
{
    if((buff == 0) || (subtype>=GYRO_NUM))
    {
        return -1;
    }
    else
    {
        memcpy(buff,&s_gyro_data_at[subtype],sizeof(GYRO_Data_t));
        return 0;
    }
}


/**
 *******************************************************************************
 * \fn          int32_t dev_gyrotel_handle(uint8_t* buff, uint8_t size)
 * \brief       ��������ң������.
 * \note        ��������ң������.
 * \param[in]   buff     ����.
 * \param[in]   size     �������ݳ���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_gyrotel_handle(uint8_t* buff, uint8_t size)
{
    int16_t temp=0;
    int32_t x=0;
    int32_t y=0;
    int32_t z=0;
    int32_t t=0;
    int32_t ms=0;
    uint8_t sendbufff[32] = {0};
    timestamp_t time;
    clock_get_time(&time);
    t = time.tv_sec;
    ms = (time.tv_usec+500)/1000;
    __disable_interrupt();      /*��֤������������ͬʱ���µ�*/    
    x =  s_gyro_data_at[0].x_speed;
    y =  s_gyro_data_at[0].y_speed;
    z =  s_gyro_data_at[0].z_speed;
    temp = s_gyro_data_at[0].x_temp;
    __enable_interrupt(); 
    /*���յ�ң������ת����PC*/
  
    /*
     *���յ�ң��������Ӧң���
     *ң������  0x22, 0x01, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55
     *����ʱ��  UTC xx xx xx xx  55 55 55 55 
     **/
    if(size == 8)
    {
        if((buff[0]==0x22) && (buff[1]==0x01))
        {
            Num_C++;
            /*��֡*/
            sendbufff[0] = 0x00;
            sendbufff[1] = 0x1D;
            sendbufff[2] = 0x35;
            sendbufff[3] = 0x01;
            sendbufff[4] = 0xA0;
            sendbufff[5] = Num_C;
            sendbufff[6] = buff[1];
            sendbufff[7] = 0;
            /*�ڶ�֡*/
            sendbufff[8] = Num_RC;
            buffer_set_int16(&sendbufff[9],temp); 
            sendbufff[11] = 0;
            sendbufff[12] = (uint8_t)(x>>16);
            sendbufff[13] = (uint8_t)(x>>8);
            sendbufff[14] = (uint8_t)(x>>0);
            sendbufff[15] = (uint8_t)(y>>16);
            /*����֡*/
            sendbufff[16] = (uint8_t)(y>>8);
            sendbufff[17] = (uint8_t)(y>>0);
            sendbufff[18] = (uint8_t)(z>>16);
            sendbufff[19] = (uint8_t)(z>>8);
            sendbufff[20] = (uint8_t)(z>>0);
            sendbufff[21] = (uint8_t)(t>>24);
            sendbufff[22] = (uint8_t)(t>>16);
            sendbufff[23] = (uint8_t)(t>>8);
            /*����֡*/
            sendbufff[24] = (uint8_t)(t>>0);
            sendbufff[25] = (uint8_t)(ms>>24);
            sendbufff[26] = (uint8_t)(ms>>16);
            sendbufff[27] = (uint8_t)(ms>>8);
            sendbufff[28] = (uint8_t)(ms>>0);
            sendbufff[29] = 0;
            sendbufff[30] = 0;
            sendbufff[31]=buffer_checksum(sendbufff,31);
            if((get_dev_state() & (1<< DEV_NUM_GYRO1))== 0)
            {
              can_tx_raw_data(OPTI_GYRO_CANID,GOM_OBC_CANID,sendbufff,32,CFP_BEGIN,1,100);
            }
        }
        else
        {
            /*����ʱ��ָ��
             * ����  0xA0 Num_C LCmd_ID St_Cmd Num_RC ֵ��Ϊ 0x55
             */
            t = buffer_get_int32(&buff[0]);
            clock_get_time(&time);
            time.tv_sec = t;
            clock_set_time(&time);
            Num_C++;
            sendbufff[0] = 0xA0;
            sendbufff[1] = Num_C;
            sendbufff[2] = LCmd_ID;
            sendbufff[3] = St_Cmd;
            sendbufff[4] = Num_RC;
            sendbufff[5] = 0x55;
            sendbufff[6] = 0x55;
            sendbufff[7] = 0x55;
            if((get_dev_state() & (1<< DEV_NUM_GYRO1))== 0)
            {
              can_tx_raw_data(OPTI_GYRO_CANID,GOM_OBC_CANID,sendbufff,8,CFP_SINGLE,1,10);
            }
            
        }
    }
    else
    {
        /*��������*/
        Num_RC++;
    }
    return 0;
}

