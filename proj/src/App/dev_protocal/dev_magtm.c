#include <stdint.h>
#include <string.h>
#include "common.h"
#include "can.h"
#include "osapi_freertos.h"
#include "dev_magtm.h"
#include "obc_protocal.h"
#include "pc_protocal.h"

MAGTM_Data_t s_magtm_data_at[MAGTM_NUM];  /*记录PC发过来的数据*/

/**
 *******************************************************************************
 * \fn          int32_t dev_magtmdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       处理磁强计子块数据.
 * \note        处理磁强计子块数据.
 * \param[in]   buff     子块内容(不包括子块长度 子块类型 子块子类型).
 * \param[in]   subtype  子块子类型 一般多个同类型设备代表设备序号.
 * \param[in]   size     子块内容长度(不包括子块长度 子块类型 子块子类型).
 * \retval      0 成功
 * \retval      其他值 失败
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
            __disable_interrupt();  /*保证读三轴数据是同时更新的*/
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
 * \brief       获取磁强计数据.
 * \note        .
 * \param[in]   buff     存储获取到的数据.
 * \param[in]   subtype  0-n 多个设备指定是哪一个.
 * \retval      0 成功
 * \retval      其他值 失败
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
 * \brief       处理磁强计遥测请求.
 * \note        处理磁强计遥测请求.
 * \param[in]   buff     数据.
 * \param[in]   size     数据内容长度.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_magtmtel_handle(uint8_t* buff, uint8_t size)
{
    uint8_t sendbufff[6] = {0};
    int32_t mx;
    int32_t my;
    int32_t mz;
    /*接收到遥测命令转发给PC*/
  

    /*  接收到遥测请求响应遥测包
     *  0000c200 2A 40 40 51 0D 08 [55 55]
     *  X  00c00200 BB A3 A2 A1 A0 XX（和校验位） 
     *  Y  00c00200 CC A3 A2 A1 A0 XX（和校验位） 
     *  Z  00c00200 DD A3 A2 A1 A0 XX（和校验位） 
     */
    if((size >= 6) && (buffer_checksum(buff,5) == buff[5]))
    {
        __disable_interrupt();      /*保证读三轴数据是同时更新的*/
        mx = s_magtm_data_at[0].mx;
        my = s_magtm_data_at[0].my;
        mz = s_magtm_data_at[0].mz;
        __enable_interrupt();
        /*X轴*/
        sendbufff[0] = 0xBB;
        buffer_set_int32(&sendbufff[1],mx);  /*非pack 32位读写mx是原子操作 这里不再做临界段保护*/
        sendbufff[5] = buffer_checksum(sendbufff,5);
        if((get_dev_state() & (1<< DEV_NUM_MAGTM1))== 0)
        {
          can_tx_raw_data(EXT_MAGTM_CANID,GOM_OBC_CANID,sendbufff,6,CFP_SINGLE,1,10);
        }
    
        /*Y轴*/
        sendbufff[0] = 0xCC;
        buffer_set_int32(&sendbufff[1],my);
        sendbufff[5] = buffer_checksum(sendbufff,5);
        if((get_dev_state() & (1<< DEV_NUM_MAGTM1))== 0)
        {
          can_tx_raw_data(EXT_MAGTM_CANID,GOM_OBC_CANID,sendbufff,6,CFP_SINGLE,1,10);
        }
        /*Z轴*/
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
        /*错误内容*/
    }
    return 0;
}

