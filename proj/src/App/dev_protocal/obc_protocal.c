#include <stdint.h>
#include <string.h>
#include "can.h"
#include "pc_protocal.h"
#include "obc_protocal.h"
#include "dev_gps.h"
#include "dev_gyro.h"
#include "dev_magtm.h"
#include "dev_ssoc.h"
#include "dev_starsensor.h"
#include "dev_wheel.h"

/**
 *******************************************************************************
 * \fn          int32_t obc_protocol_handle_data(uint8_t descanid, uint8_t srccanid, uint8_t* buff, uint32_t size)
 * \brief       OBC发过来的数据帧处理.
 * \note        处理数据帧内容.
 * \param[in]   descanid  目的id.
 * \param[in]   srccanid  源id.
 * \param[in]   buff     指向数据内容.
 * \param[in]   size     数据的长度.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t obc_protocol_handle_data(uint8_t canid, uint8_t srccanid, uint8_t* buff, uint32_t size)
{
    /*canid 接收时一斤检查 现在不需要再检查*/
    uint16_t datalen = 0;
    uint8_t* p=0;
    int32_t res =0;
    if((buff ==0) || (size == 0))
    {
        return -1;
    }
    datalen = ((uint16_t)buff[0]<<8) | (uint16_t)buff[1];  /*长度*/
    if((size>8) && (((datalen+3) != size) || (buffer_checksum(buff,size-1) != buff[size-1])))
    {
        return -1; /*长度或者校验错*/
    }
    /*有效数据长度 处理数据*/
    if(size > 8)
    {
        size -= 3;
        p = &buff[2];
    }
    else
    {
        size -= 0;
        p = &buff[0];
    }
    switch(canid)
    {
        case GPS_CANID:
            dev_gpstel_handle(p,size);
        break;
        case EXT_MAGTM_CANID:
            dev_magtmtel_handle(p,size);
        break;
        case WHEEL_X_CANID:
            dev_wheeltel_handle(p,0,size);
        break;
        case WHEEL_Y_CANID:
            dev_wheeltel_handle(p,1,size);
        break;
        case WHEEL_Z_CANID:
            dev_wheeltel_handle(p,2,size);
        break;
        case WHEEL_A_CANID:
            dev_wheeltel_handle(p,3,size);
        break;
        case STAR_SENSOR_CANID:
            dev_starsensortel_handle(p,0,size);
        break;
        case STAR_SENSOR2_CANID:
            dev_starsensortel_handle(p,1,size);
        break;
        case STAR_SENSOR3_CANID:
            dev_starsensortel_handle(p,2,size);
        break;
        case OPTI_GYRO_CANID:
            dev_gyrotel_handle(p,size);
        break;
        case GOM_OBC_CANID:
            /*设备返回的遥测数据*/
            dev_wheeltel_tlhandle(p,srccanid,size);
        break;
        default:
            res = -1;
        break;              
    }
    return res;
}