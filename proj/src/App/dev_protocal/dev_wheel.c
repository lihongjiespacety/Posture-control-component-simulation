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

WHEEL_Data_t s_wheel_data_at[WHEEL_NUM]={0};     /*记录PC发过来的数据*/
WHEEL_Data_t s_wheel_obcdata_at[WHEEL_NUM]={0};  /*记录OBC发过来的反馈数据 和实际的轮子反馈的实际速度*/

static uint8_t Num_T[4] =  {0};    /*遥测正确帧计数*/
static uint8_t SetNetNum_T[4] =  {0};   
static uint8_t SetSpeedNum_T[4] =  {0};   
static uint8_t LCmd_ID[4] =  {0};  /*最近执行指令吗*/
static uint8_t Num_RC[4] =  {0};  /*错误帧计数*/
static uint8_t CAN_ID[4] =  {WHEEL_X_CANID,WHEEL_Y_CANID,WHEEL_Z_CANID,WHEEL_A_CANID};   /*ID*/

/**
 *******************************************************************************
 * \fn          int32_t dev_wheeldata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       处理飞轮子块数据.
 * \note        处理飞轮子块数据.
 * \param[in]   buff     子块内容(不包括子块长度 子块类型 子块子类型).
 * \param[in]   subtype  子块子类型 一般多个同类型设备代表设备序号.
 * \param[in]   size     子块内容长度(不包括子块长度 子块类型 子块子类型).
 * \retval      0 成功
 * \retval      其他值 失败
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
            __disable_interrupt();      /*保证读数据是同时更新的*/    
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
 * \brief       获取飞轮数据.
 * \note        .
 * \param[in]   buff     存储获取到的数据.
 * \param[in]   subtype  0-n 多个设备指定是哪一个.
 * \retval      0 成功
 * \retval      其他值 失败
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
 * \brief       处理飞轮遥测请求.
 * \note        处理飞轮遥测请求.
 * \param[in]   buff     数据.
 * \param[in]   subtype  一般多个同类型设备代表设备序号.
 * \param[in]   size     数据内容长度.
 * \retval      0 成功
 * \retval      其他值 失败
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
    
    /*接收到遥测命令转发给PC*/
  
    /*接收到遥测请求响应遥测包
     *      0x20, 0x01, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55
     */
    __disable_interrupt();      /*保证读数据是同时更新的*/    
    speed = s_wheel_data_at[subtype].speed;
    toq = s_wheel_data_at[subtype].toq;
    mode = s_wheel_data_at[subtype].mode;
    __enable_interrupt(); 
    
    /*接收到遥测命令转发给PC*/

    if(size == 8)
    {
      
        if(buff[0] == 0x21)
        {
            /*设置最大力矩  将数据通过串口转发到PC*/ 
            s_wheel_obcdata_at[subtype].mode = s_wheel_data_at[subtype].mode;
            memcpy(&(s_wheel_obcdata_at[subtype].toq),&buff[1],4);    /*传过来的是大端 直接大端发送*/
        }
        else if(buff[0] == 0x22)
        {
            /*设置速度*/
            s_wheel_obcdata_at[subtype].mode = s_wheel_data_at[subtype].mode;
            memcpy(&(s_wheel_obcdata_at[subtype].speed),&buff[1],4);    /*传过来的是大端 直接大端发送*/
            SetSpeedNum_T[subtype]++;
            ///pc_protocol_initbuffer(sendbufff, &topcsize,sizeof(sendbufff));
            ///pc_protocol_apendbuffer(sendbufff,&topcsize,sizeof(sendbufff),&s_wheel_obcdata_at[subtype],sizeof(WHEEL_Data_t),DATA_WHEEL,subtype);
            ///driver_uart_send(HAL_UART_4, sendbufff, topcsize, 10, &erro);
        }
        else if(buff[0] == 0x23)
        {
            /*设置电流*/

          
        }
        else if(buff[0] == 0x24)
        {
            /*设置net力矩*/
            s_wheel_obcdata_at[subtype].mode = s_wheel_data_at[subtype].mode;
            __disable_interrupt();      /*保证读数据是同时更新的*/    
            memcpy(&(s_wheel_obcdata_at[subtype].toq),&buff[1],4);    
            __enable_interrupt(); 
            SetNetNum_T[subtype]++;
          
        }
        else if(buff[0] == 0xFE)
        {
            /*设置485*/

          
        }
        else if(buff[0] == 0x2C)
        {
            /*设置can复位*/
          
        }
        if(buff[0] == 0xD2)
        {
            /*获取版本信息*/
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
            /*接收到遥测请求响应遥测包*/
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
 * \brief       处理飞轮遥测包.
 * \note        处理飞轮遥测包.
 * \param[in]   buff     数据. 指向数据包中去掉前两字节长度后的位置
 * \param[in]   canid  id.
 * \param[in]   size     数据内容长度.
 * \retval      0 成功
 * \retval      其他值 失败
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