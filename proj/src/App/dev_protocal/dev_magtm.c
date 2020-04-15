#include <stdint.h>
#include <string.h>
#include "common.h"
#include "can.h"
#include "osapi_freertos.h"
#include "dev_magtm.h"
#include "obc_protocal.h"
#include "pc_protocal.h"

MAGTM_Data_t s_magtm_data_at[MAGTM_NUM];  /*��¼PC������������*/

/**
 *******************************************************************************
 * \fn          int32_t dev_magtmdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       �����ǿ���ӿ�����.
 * \note        �����ǿ���ӿ�����.
 * \param[in]   buff     �ӿ�����(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \param[in]   subtype  �ӿ������� һ����ͬ�����豸�����豸���.
 * \param[in]   size     �ӿ����ݳ���(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_magtmdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
{
    if((buff == 0) || (subtype>=MAGTM_NUM))
    {
        return -1;
    }
    else
    {
        if(size == sizeof(MAGTM_Data_t))
        {
            __disable_interrupt();  /*��֤������������ͬʱ���µ�*/
            s_magtm_data_at[subtype].mx  = buffer_get_int32(&buff[0]);   
            s_magtm_data_at[subtype].my  = buffer_get_int32(&buff[4]);
            s_magtm_data_at[subtype].mz  = buffer_get_int32(&buff[8]);
            __enable_interrupt();
            OsPrintf(OSAPI_DEBUG_INFO,"get magtmdata\r\n");
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
 * \fn          int32_t dev_magtmdata_get(MAGTM_Data_t* buff,uint8_t subtype))
 * \brief       ��ȡ��ǿ������.
 * \note        .
 * \param[in]   buff     �洢��ȡ��������.
 * \param[in]   subtype  0-n ����豸ָ������һ��.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_magtmdata_get(MAGTM_Data_t* buff,uint8_t subtype)
{
    if((buff == 0) || (subtype>=MAGTM_NUM))
    {
        return -1;
    }
    else
    {
        memcpy(buff,&s_magtm_data_at[subtype],sizeof(MAGTM_Data_t));
        return 0;
    }
}

/**
 *******************************************************************************
 * \fn          int32_t dev_magtmtel_handle(uint8_t* buff, uint8_t size)
 * \brief       �����ǿ��ң������.
 * \note        �����ǿ��ң������.
 * \param[in]   buff     ����.
 * \param[in]   size     �������ݳ���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_magtmtel_handle(uint8_t* buff, uint8_t size)
{
    uint8_t sendbufff[6] = {0};
    int32_t mx;
    int32_t my;
    int32_t mz;
    /*���յ�ң������ת����PC*/
  

    /*  ���յ�ң��������Ӧң���
     *  0000c200 2A 40 40 51 0D 08 [55 55]
     *  X  00c00200 BB A3 A2 A1 A0 XX����У��λ�� 
     *  Y  00c00200 CC A3 A2 A1 A0 XX����У��λ�� 
     *  Z  00c00200 DD A3 A2 A1 A0 XX����У��λ�� 
     */
    if((size >= 6) && (buffer_checksum(buff,5) == buff[5]))
    {
        __disable_interrupt();      /*��֤������������ͬʱ���µ�*/
        mx = s_magtm_data_at[0].mx;
        my = s_magtm_data_at[0].my;
        mz = s_magtm_data_at[0].mz;
        __enable_interrupt();
        /*X��*/
        sendbufff[0] = 0xBB;
        buffer_set_int32(&sendbufff[1],mx);  /*��pack 32λ��дmx��ԭ�Ӳ��� ���ﲻ�����ٽ�α���*/
        sendbufff[5] = buffer_checksum(sendbufff,5);
        if((get_dev_state() & (1<< DEV_NUM_MAGTM1))== 0)
        {
          can_tx_raw_data(EXT_MAGTM_CANID,GOM_OBC_CANID,sendbufff,6,CFP_SINGLE,1,10);
        }
    
        /*Y��*/
        sendbufff[0] = 0xCC;
        buffer_set_int32(&sendbufff[1],my);
        sendbufff[5] = buffer_checksum(sendbufff,5);
        if((get_dev_state() & (1<< DEV_NUM_MAGTM1))== 0)
        {
          can_tx_raw_data(EXT_MAGTM_CANID,GOM_OBC_CANID,sendbufff,6,CFP_SINGLE,1,10);
        }
        /*Z��*/
        sendbufff[0] = 0xDD;
        buffer_set_int32(&sendbufff[1],mz);
        sendbufff[5] = buffer_checksum(sendbufff,5);
        if((get_dev_state() & (1<< DEV_NUM_MAGTM1))== 0)
        {
          can_tx_raw_data(EXT_MAGTM_CANID,GOM_OBC_CANID,sendbufff,6,CFP_SINGLE,1,10);
        }
    }
    else
    {
        /*��������*/
    }
    return 0;
}

