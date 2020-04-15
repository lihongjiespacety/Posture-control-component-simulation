#include <stdint.h>
#include <string.h>
#include "common.h"
#include "osapi_freertos.h"
#include "dev_starsensor.h"
#include "can.h"
#include "obc_protocal.h"
#include "pc_protocal.h"

STARSENSOR_Data_t s_starsensor_data_at[STARSENSOR_NUM];  /*记录PC发过来的数据*/

static uint8_t Num_C = 0;    /*命令正确帧计数*/
static uint8_t Num_T = 0;    /*遥测正确帧计数*/
static uint8_t LCmd_ID = 0;  /*最近执行指令吗*/
static uint8_t Num_RC = 0;  /*错误帧计数*/

/**
 *******************************************************************************
 * \fn          int32_t dev_starsensordata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       处理星敏子块数据.
 * \note        处理星敏子块数据.
 * \param[in]   buff     子块内容(不包括子块长度 子块类型 子块子类型).
 * \param[in]   subtype  子块子类型 一般多个同类型设备代表设备序号.
 * \param[in]   size     子块内容长度(不包括子块长度 子块类型 子块子类型).
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_starsensordata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
{
    if((buff == 0) || (subtype>=STARSENSOR_NUM))
    {
        return -1;
    }
    else
    {
        if(size == sizeof(STARSENSOR_Data_t))
        {
            __disable_interrupt();      /*保证读数据是同时更新的*/    
            s_starsensor_data_at[subtype].i  = buffer_get_int32(&buff[0]);
            s_starsensor_data_at[subtype].j  = buffer_get_int32(&buff[4]);
            s_starsensor_data_at[subtype].k  = buffer_get_int32(&buff[8]);
            s_starsensor_data_at[subtype].q  = buffer_get_int32(&buff[12]);
            s_starsensor_data_at[subtype].x_speed = buffer_get_int32(&buff[16]);
            s_starsensor_data_at[subtype].y_speed = buffer_get_int32(&buff[20]);
            s_starsensor_data_at[subtype].z_speed = buffer_get_int32(&buff[24]);
            s_starsensor_data_at[subtype].exposure = buff[28];
            s_starsensor_data_at[subtype].state = buff[29];
            s_starsensor_data_at[subtype].sec  = buffer_get_int32(&buff[30]);
            s_starsensor_data_at[subtype].us   = buffer_get_int32(&buff[34]);
            __enable_interrupt(); 
    
            OsPrintf(OSAPI_DEBUG_INFO,"get starsensordata\r\n");
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
 * \fn          int32_t dev_starsensordata_get(STARSENSOR_Data_t* buff,uint8_t subtype))
 * \brief       获取星敏数据.
 * \note        .
 * \param[in]   buff     存储获取到的数据.
 * \param[in]   subtype  0-n 多个设备指定是哪一个.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_starsensordata_get(STARSENSOR_Data_t* buff,uint8_t subtype)
{
    if((buff == 0) || (subtype>=STARSENSOR_NUM))
    {
        return -1;
    }
    else
    {
        memcpy(buff,&s_starsensor_data_at[subtype],sizeof(STARSENSOR_Data_t));
        return 0;
    }
}

/**
 *******************************************************************************
 * \fn          int32_t dev_starsensortel_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       处理星敏遥测请求.
 * \note        处理星敏遥测请求.
 * \param[in]   buff     数据.
 * \param[in]   subtype  一般多个同类型设备代表设备序号.
 * \param[in]   size     数据内容长度.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_starsensortel_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
{
     uint8_t sendbufff[57] = {0};
     int32_t q1;
     int32_t q2;
     int32_t q3;
     int32_t q4;
     uint32_t sec;
     uint32_t us;
     uint8_t exposure;
     uint8_t state;
     int32_t x_speed;
     int32_t y_speed;
     int32_t z_speed;
    __disable_interrupt();      /*保证读数据是同时更新的*/    
    q1 =  s_starsensor_data_at[subtype].i;
    q2 =  s_starsensor_data_at[subtype].j;
    q3 =  s_starsensor_data_at[subtype].k;
    q4 =  s_starsensor_data_at[subtype].q;
    sec = s_starsensor_data_at[subtype].sec;
    us = s_starsensor_data_at[subtype].us;
    x_speed = s_starsensor_data_at[subtype].x_speed;
    y_speed = s_starsensor_data_at[subtype].y_speed;
    z_speed = s_starsensor_data_at[subtype].z_speed;
    exposure = s_starsensor_data_at[subtype].exposure;
    state = s_starsensor_data_at[subtype].state;
    __enable_interrupt(); 
    
    /*接收到遥测命令转发给PC*/
  
    /*接收到遥测请求响应遥测包*/
    if((size == 2) && (buff[0] == 0x00)  && (buff[1] == 0xFF))
    {
        Num_T++;
        LCmd_ID = 0;
        
        sendbufff[0] = 0x00;
        sendbufff[1] = 0x36;
        sendbufff[2] = 0x35;
        sendbufff[3] = 0xFF;
        sendbufff[4] = Num_T;
        sendbufff[5] = Num_C;
        sendbufff[6] = Num_RC;
        sendbufff[7] = LCmd_ID;
        
        buffer_set_int32(&sendbufff[8],q1);
        buffer_set_int32(&sendbufff[12],q2);
        buffer_set_int32(&sendbufff[16],q3);
        buffer_set_int32(&sendbufff[20],q4);
        
        sendbufff[32] = exposure;
        
        sendbufff[47] = (uint8_t)(x_speed>>16);
        sendbufff[48] = (uint8_t)(x_speed>>8);
        sendbufff[49] = (uint8_t)(x_speed>>0);
        sendbufff[50] = (uint8_t)(y_speed>>16);
        sendbufff[51] = (uint8_t)(y_speed>>8);
        sendbufff[52] = (uint8_t)(y_speed>>0);
        sendbufff[53] = (uint8_t)(z_speed>>16);
        sendbufff[54] = (uint8_t)(z_speed>>8);
        sendbufff[55] = (uint8_t)(z_speed>>0);
        
        buffer_set_uint32(&sendbufff[24],sec);
        sendbufff[28] = (uint8_t)(us>>16);
        sendbufff[29] = (uint8_t)(us>>8);
        sendbufff[30] = (uint8_t)(us>>0);
           
        sendbufff[38] = state;
        sendbufff[56]=buffer_checksum(sendbufff,56);
        if((get_dev_state() & ((uint32_t)1<< (DEV_NUM_STARSENSOR1+subtype)))== 0)
        {
          if(subtype==0)
          {
            can_tx_raw_data(STAR_SENSOR_CANID,GOM_OBC_CANID,sendbufff,57,CFP_BEGIN,1,100);
          }
          else if(subtype==1)
          {
            can_tx_raw_data(STAR_SENSOR2_CANID,GOM_OBC_CANID,sendbufff,57,CFP_BEGIN,1,100);
          }
          else if(subtype==2)
          {
            can_tx_raw_data(STAR_SENSOR3_CANID,GOM_OBC_CANID,sendbufff,57,CFP_BEGIN,1,100);
          }
        }
    }
    else
    {
        Num_RC++;
    }

    return 0;
}

