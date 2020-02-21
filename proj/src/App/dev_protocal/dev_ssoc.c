#include <stdint.h>
#include <string.h>
#include "common.h"
#include "osapi_freertos.h"
#include "dev_ssoc.h"

SSOC_Data_t s_ssoc_data_at[SSOC_NUM];  /*��¼PC������������*/

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
            s_ssoc_data_at[subtype].erfa  = buffer_get_int32(&buff[0]);
            s_ssoc_data_at[subtype].beta  = buffer_get_int32(&buff[4]);
            s_ssoc_data_at[subtype].sta   = buff[5];
            s_ssoc_data_at[subtype].cell1 = buffer_get_int32(&buff[6]);
            s_ssoc_data_at[subtype].cell2 = buffer_get_int32(&buff[10]);
            s_ssoc_data_at[subtype].cell3 = buffer_get_int32(&buff[14]);
            s_ssoc_data_at[subtype].cell4 = buffer_get_int32(&buff[18]);
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
 * \fn          int32_t dev_ssocdata_get(SSOC_Data_t* buff,uint8_t subtype))
 * \brief       ��ȡ̫������.
 * \note        .
 * \param[in]   buff     �洢��ȡ��������.
 * \param[in]   subtype  0-n ����豸ָ������һ��.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_ssocdata_get(SSOC_Data_t* buff,uint8_t subtype)
{
    if((buff == 0) || (subtype>=SSOC_NUM))
    {
        return -1;
    }
    else
    {
        memcpy(buff,&s_ssoc_data_at[subtype],sizeof(SSOC_Data_t));
        return 0;
    }
}

/**
 *******************************************************************************
 * \fn          int32_t dev_ssoctel_handle(uint8_t* buff, uint8_t size)
 * \brief       ����̫��ң������.
 * \note        ����̫��ң������.
 * \param[in]   buff     ����.
 * \param[in]   size     �������ݳ���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_ssoctel_handle(uint8_t* buff, uint8_t size)
{
    /*���յ�ң������ת����PC*/
  
    /*���յ�ң��������Ӧң���*/


    return 0;
}

