#include <stdint.h>
#include <string.h>
#include "common.h"
#include "osapi_freertos.h"
#include "dev_ssoc.h"

SSOC_Data_t s_ssoc_data_at[SSOC_NUM];  /*记录PC发过来的数据*/

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
 * \brief       获取太敏数据.
 * \note        .
 * \param[in]   buff     存储获取到的数据.
 * \param[in]   subtype  0-n 多个设备指定是哪一个.
 * \retval      0 成功
 * \retval      其他值 失败
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
 * \brief       处理太敏遥测请求.
 * \note        处理太敏遥测请求.
 * \param[in]   buff     数据.
 * \param[in]   size     数据内容长度.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_ssoctel_handle(uint8_t* buff, uint8_t size)
{
    /*接收到遥测命令转发给PC*/
  
    /*接收到遥测请求响应遥测包*/


    return 0;
}

