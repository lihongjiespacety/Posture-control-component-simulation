#include <stdint.h>
#include <string.h>
#include "driver_uart.h"
#include "osapi_freertos.h"
#include "dev_magtm.h"
#include "dev_starsensor.h"
#include "dev_gps.h"
#include "dev_ssoc.h"
#include "dev_gyro.h"
#include "dev_wheel.h"
#include "dev_magt.h"
#include "pc_protocal.h"
#include "common.h"

#define READ_BUFF_MAX 256   /*读一次串口大小*/
#define READ_TIMEOUT 200   /*读一次串口超时时间*/

#define FRAME_MINI_SIZE 6   /*一帧数据最小长度*/
#define FRAME_HEAD_SIZE 5   /*帧头长度*/
#define FRAME_BUFF_MAX 512  /*帧处理缓冲区大小*/

#define HEAD_FLAG1 0xAA
#define HEAD_FLAG2 0x55


static uint8_t s_framebuff_au8[FRAME_BUFF_MAX];   /*帧处理缓冲区*/
static uint32_t s_framebuff_size=0;              /*帧缓冲区中数据长度*/


/**
 *  模拟数据帧                 磁强计  10001nT 10002nT 10003nT              星敏  (0i+0j+0k,1)   10000S  2mS                                                                                           太敏  (0i+0j+0k,1)   10000S  2mS
 *  数据帧      AA 55 00 9D 00 0F 01 00 00 00 27 11 00 00 27 12 00 00 27 13 29 02 00 00 00 00 00 00 00 00 00 00 00 00 00 7F FF FF FF 00 00 27 11 00 00 00 31 00 00 00 00 00 00 00 00 00 00 00 00 00 00 1C 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
 *                             GPS                                                                        20年2月18日 15时43分01秒900mS 陀螺                                                          飞轮                                  校验和
 *                             23 04 00 00 00 27 11 00 00 27 12 00 00 27 13 00 00 27 11 00 00 27 12 00 00 27 13 14 02 12 0F 2B 01 03 84 15 05 00 00 00 27 11 00 00 27 12 00 00 27 13 07 D0 07 D0 07 D0 0C 06 00 13 88 13 88 13 88 13 88 00  B5  
 *                             磁力矩器 
 *
 * AA 55 00 9E 00 0F 01 00 00 00 27 11 00 00 27 12 00 00 27 13 29 02 00 00 00 00 00 00 00 00 00 00 00 00 00 7F FF FF FF 00 00 27 11 00 00 00 31 00 00 00 00 00 00 00 00 00 00 00 00 00 00 1C 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 23 04 00 00 00 27 11 00 00 27 12 00 00 27 13 00 00 27 11 00 00 27 12 00 00 27 13 14 02 12 0F 2B 01 03 84 15 05 00 00 00 27 11 00 00 27 12 00 00 27 13 07 D0 07 D0 07 D0 0C 06 00 13 88 13 88 13 88 13 88 00 B6
 *  命令帧     AA 55 xx xx 01
 */

/**
 *******************************************************************************
 * \fn          static int32_t pc_protocol_gethead(uint8_t* buff, uint32_t* index)
 * \brief       搜寻帧头标志.
 * \note        查找帧头标志HEAD_FLAG1 HEAD_FLAG2.
 * \param[in]   buff     指向帧缓冲区数据.
 * \param[in]   index    指向索引计数.
 * \retval      >=0      查找到的索引位置
 * \retval      <0       查找失败
 *******************************************************************************
 */
static int32_t pc_protocol_gethead(uint8_t* buff, uint32_t* index)
{
    uint32_t i = 0;
    int32_t position = -1;
    if(*index < 2)
    {
        return -1;  /*小于最小帧长度 直接返回*/
    }
    for(i=0; i<(*index-1); i++)
    {
        if((buff[i] == HEAD_FLAG1) && (buff[i+1] == HEAD_FLAG2))
        {
            position = i; 
            break;
        }
    }
    return position; 
}

/**
 *******************************************************************************
 * \fn          static int32_t pc_protocol_handle_data(uint8_t* buff, uint32_t size)
 * \brief       PC发过来的数据帧处理.
 * \note        处理数据帧内容.
 * \param[in]   buff     指向子块内容不包括帧标志 长度 类型 校验和 buff可能被改写.
 * \param[in]   size     子块内容长度不包括帧标志 长度 类型 校验和.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
static int32_t pc_protocol_handle_data(uint8_t* buff, uint32_t size)
{
    uint16_t datalen = 0;
    int32_t res = 0;   /*处理状态  1需要继续处理 0不需要继续处理成功 -1不需要继续处理失败*/
    do
    {
        datalen = buff[0];
        if(size > 0)
        {
            if(size >= datalen)
            {
                /*有效数据长度 处理数据*/
                switch(buff[1])
                {
                    case DATA_MAGTM:
                        res = dev_magtmdata_handle(&buff[3], buff[2], datalen-3);  /*传递子块内容和长度(不包括子块长度 子块类型 子块子类型)*/
                    break;
                    case DATA_STARSENSOR:
                        res = dev_starsensordata_handle(&buff[3], buff[2], datalen-3);  /*传递子块内容和长度(不包括子块长度 子块类型 子块子类型)*/
                    break;
                    case DATA_SSOC:
                        res = dev_ssocdata_handle(&buff[3], buff[2], datalen-3);  /*传递子块内容和长度(不包括子块长度 子块类型 子块子类型)*/ 
                    break;
                    case DATA_GPS:
                        res = dev_gpsdata_handle(&buff[3], buff[2], datalen-3);  /*传递子块内容和长度(不包括子块长度 子块类型 子块子类型)*/
                    break;
                    case DATA_GYRO:
                        res = dev_gyrodata_handle(&buff[3], buff[2], datalen-3);  /*传递子块内容和长度(不包括子块长度 子块类型 子块子类型)*/
                    break;
                    case DATA_WHEEL:
                        res = dev_wheeldata_handle(&buff[3], buff[2], datalen-3);  /*传递子块内容和长度(不包括子块长度 子块类型 子块子类型)*/
                    break;
                    default:
                        res = -1;
                    break;
                
                }
                /*调整数据继续处理*/
                if(res == 0)
                {
                    memmove(buff,buff+datalen,size - datalen);
                    size -= datalen;
                    res = 1;
                }
                else
                {
                    /*处理失败*/
                    res = -1;
                }
            }
            else
            {
                /*无效数据长度*/
                res = -1;
            }
        }
        else
        {
            res = 0;   /*数据处理完*/
        }
    }while(res == 1);  /*如果需要继续处理从头开始*/

    return res;
}

/**
 *******************************************************************************
 * \fn          static int32_t pc_protocol_handle_cmd(uint8_t* buff, uint32_t size)
 * \brief       PC发过来的命令帧处理.
 * \note        处理命令帧内容.
 * \param[in]   buff     指向子块内容不包括帧标志 长度 类型 校验和 buff可能被改写.
 * \param[in]   size     子块内容长度不包括帧标志 长度 类型 校验和.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
static int32_t pc_protocol_handle_cmd(uint8_t* buff, uint32_t size)
{
    uint16_t datalen = 0;
    int32_t res = 0;   /*处理状态  1需要继续处理 0不需要继续处理成功 -1不需要继续处理失败*/
    do
    {
        datalen = buff[0];
        if(size > 0)
        {
            if(size >= datalen)
            {
                /*有效数据长度 处理数据*/
                switch(buff[1])
                {
                    case 0:
                    break;
                    case 1:
                    break;
                
                }
                /*调整数据继续处理*/
                memmove(buff,buff+datalen,size - datalen);
                size -= datalen;
                res = 1;
            }
            else
            {
                /*无效数据长度*/
                res = -1;
            }
        }
        else
        {
            res = 0;   /*数据处理完*/
        }
    }while(res == 1);  /*如果需要继续处理从头开始*/

    return res;
}



/**
 *******************************************************************************
 * \fn          static int32_t pc_protocol_handle(uint8_t* buff, uint32_t* size)
 * \brief       PC发过来的帧数据处理.
 * \note        接收到串口数据后调用该函数处理.
 * \param[in]   buff     指向帧缓冲区数据,内容可能被改写.
 * \param[in]   size     帧数据长度,内容可能会被改写.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
static int32_t pc_protocol_handle(uint8_t* buff, uint32_t* size)
{
    int32_t headposition = -1;
    int32_t res = 0;   /*处理状态  1需要继续处理 0不需要继续处理成功 -1不需要继续处理失败*/
    uint16_t datalen = 0;
    if(*size <= 6)
    {
        return 0;  /*小于最小帧长度 直接返回*/
    }else{}
    
    do
    {
        headposition = pc_protocol_gethead(buff,size);
        if(headposition >= 0)
        {
            /*调整删除前面无效的数据*/
            if(headposition > 0)
            {
                memmove(buff,buff+headposition,*size- headposition);
                *size -= headposition;
            }
            else
            {
                /*headposition == 0不需要调整*/
            }
            
            if(*size > FRAME_MINI_SIZE)
            {
                datalen = ((uint16_t)buff[2]<<8) | (uint16_t)buff[3];  /*长度*/
                if((datalen<=FRAME_BUFF_MAX) && (datalen>=FRAME_MINI_SIZE))
                {
                    if(*size >= datalen)
                    {
                        /*数据已收全*/
                        if(buffer_checksum(buff, datalen-1) == buff[datalen-1])
                        {
                             /*数据校验成功  处理数据*/
                            OsPrintf(OSAPI_DEBUG_INFO,"get frame type=%d framelen=%d\r\n", buff[4],datalen);
                            switch(buff[4])
                            {
                                case 0:
                                    pc_protocol_handle_data(buff+FRAME_HEAD_SIZE,datalen-FRAME_MINI_SIZE);  /*处理数据帧*/
                                    pc_protocol_ackhandle(); /*发送返回帧数据*/
                                break;
                                case 1:
                                    pc_protocol_handle_cmd(buff+FRAME_HEAD_SIZE,datalen-FRAME_MINI_SIZE);  /*处理命令帧*/
                                break;
                                default:
                                break;
                            }
                            memmove(buff,buff+datalen,*size - datalen);  /*删除本次已经处理完的帧数据*/
                            *size -= datalen;
                            res = 1;                            /*需要继续处理*/
                        }
                        else
                        {
                            /*数据校验错误  删掉帧头继续处理*/
                            memmove(buff,buff+2,*size - 2);   
                            *size -= 2;
                            res = 1;  /*需要继续处理*/
                        }
                    }
                    else
                    {
                        res = -1;  /*数据未收全 无需处理*/   
                    }
                
                }
                else
                {
                    /*长度错误  删掉帧头继续处理*/
                    memmove(buff,buff+2,*size - 2);   
                    *size -= 2;
                    res = 1;  /*需要继续处理*/
                }
            }
            else
            {
                res = -1;  /*数据太短无需处理*/
            }
        }
        else
        {
            /*没有找到标志  回写索引值 使得所有数据无效 从头开始*/
            *size = 0;
            res = -1;
        }
        
    }while(res == 1);  /*如果需要继续处理从头开始*/
    return res;
}



/**
 * 与PC通讯串口协议处理任务
 */
void uarttask(void *pvParameters )
{
    uint32_t readlen = 0;                   /*本次实际读到的字节数*/
    uint32_t toreadlen = 0;                 /*本次需要读的字节数*/
    uint8_t readbuff[READ_BUFF_MAX] = {0};  /*本次的读缓冲*/
    int32_t erro = 0;                       /*读错误码*/
    /*初始化与PC通讯的串口*/
    HAL_UART_CFG_t cfg;
    cfg.id = HAL_UART_4;
    cfg.baud = HAL_UART_BAUD_115200;
    cfg.datalen = HAL_UART_DATA_8;
    cfg.parity = HAL_UART_PARITY_NONE;
    cfg.stopb = HAL_UART_STOPB_1;
    driver_uart_set(&cfg);
    driver_uart_flush(HAL_UART_4);

//  cfg.id = HAL_UART_5;
//  cfg.baud = HAL_UART_BAUD_115200;
//  cfg.datalen = HAL_UART_DATA_8;
//  cfg.parity = HAL_UART_PARITY_NONE;
//  cfg.stopb = HAL_UART_STOPB_1;
//  driver_uart_set(&cfg);
//  driver_uart_flush(HAL_UART_5);
  
    while(1)
    {
        /*串口收到数据进行转发*/
        do
        {
            if((FRAME_BUFF_MAX-s_framebuff_size)>=READ_BUFF_MAX)
            {
                toreadlen = READ_BUFF_MAX;  /*如果帧缓冲区能放的下数据大于 读缓冲区 那么一次读缓冲区大小*/
            }
            else
            {
                toreadlen = FRAME_BUFF_MAX-s_framebuff_size;  /*如果帧缓冲区能放的下数据小于 读缓冲区 那么只读能存的下的大小*/
            }
            readlen = driver_uart_recv(HAL_UART_4, readbuff, toreadlen, READ_TIMEOUT, &erro);
            if(readlen > 0)
            {
                memcpy(&s_framebuff_au8[s_framebuff_size],readbuff,readlen);
                s_framebuff_size += readlen;
            }
            else
            {
                /*未读到数据继续读*/
            }
            /*调用帧处理接口*/
            pc_protocol_handle(s_framebuff_au8, &s_framebuff_size);
        }
        while(readlen);
        // OsTimeDelay(10);   // driver_uart_recv有任务延时功能 不需要再调用.
    }
}

/**
 *******************************************************************************
 * \fn          int32_t pc_protocol_initbuffer(uint8_t* buff, uint32_t* size,uint32_t bufflen)
 * \brief       打包一个和pc通讯初始化的空包.
 * \note        .
 * \param[in]   buff     指向帧缓冲区数据,内容可能被改写 调用者分配.
 * \param[in]   size     有效帧数据长度,内容可能会被改写.
 * \param[in]   bufflen  有效帧数据长度,内容可能会被改写 调用者指定.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t pc_protocol_initbuffer(uint8_t* buff, uint32_t* size,uint32_t bufflen)
{
    if((bufflen < FRAME_MINI_SIZE) || (buff==0) || (size==0))
    {
        return -1;
    }
    buff[0] = 0xAA;
    buff[1] = 0x55;
    buff[2] = 0x00;
    buff[3] = 0x06;
    buff[4] = 0x02;  /*返回数据包*/
    buff[5] = 0x02;
    *size = 6;
    return 0;
}

/**
 *******************************************************************************
 * \fn          int32_t pc_protocol_apendbuffer(uint8_t* buff, uint32_t* size,uint32_t bufflen,void* adddata,uint32_t adddatalen,uint8_t type, uint8_t subtype)
 * \brief       添加一个数据块.
 * \note        .
 * \param[in]   buff     指向帧缓冲区数据,内容可能被改写 调用者分配.
 * \param[in]   size     有效帧数据长度,内容可能会被改写.
 * \param[in]   adddata  待添加的数据.
 * \param[in]   adddatalen  待添加数据类型.
 * \param[in]   type  类型.
 * \param[in]   subtype  子类型.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t pc_protocol_apendbuffer(uint8_t* buff, uint32_t* size,uint32_t bufflen,void* adddata,uint32_t adddatalen,uint8_t type, uint8_t subtype)
{
    uint16_t len;
    if((adddatalen <= 3) || (buff==0) || (size==0) || (adddata==0))
    {
        return -1;
    }
    len = *size + adddatalen + 3;
    if(len > bufflen)
    {
        return -1;
    }
    //buff[0] = 0xAA;
    //buff[1] = 0x55;
    buff[2] = (uint8_t)(len>>8);
    buff[3] = (uint8_t)(len & 0xFF);
    //buff[4] = 0x02;  /*返回数据包*/
    
    buff[*size-1] = adddatalen+3;
    buff[*size] = type;
    buff[*size+1] = subtype;
    memcpy(&buff[*size+2],adddata,adddatalen);
    
    *size += (3+adddatalen);
    
    buff[*size-1] = buffer_checksum(buff,*size-1);
    
    return 0;
}

/**
 *******************************************************************************
 * \fn          int32_t pc_protocol_ackhandle(void)
 * \brief       PC发过来的帧数据回应处理(返回数据包).
 * \note        接收到串口数据后调用该函数处理.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t pc_protocol_ackhandle(void)
{
    uint8_t sendbufff[128] = {0};
    uint32_t topcsize=0;
    int32_t erro;
    
    MAGT_Data_t magtdata;
    WHEEL_Data_t wheeldata;
    dev_magtdata_get(&magtdata,0,1);
    pc_protocol_initbuffer(sendbufff, &topcsize,sizeof(sendbufff));
    pc_protocol_apendbuffer(sendbufff,&topcsize,sizeof(sendbufff),&magtdata,sizeof(magtdata),DATA_MAGT,0);
    
    dev_wheeldata_get(&wheeldata,0);
    pc_protocol_apendbuffer(sendbufff,&topcsize,sizeof(sendbufff),&wheeldata,sizeof(WHEEL_Data_t),DATA_WHEEL,0);
    
    dev_wheeldata_get(&wheeldata,1);
    pc_protocol_apendbuffer(sendbufff,&topcsize,sizeof(sendbufff),&wheeldata,sizeof(WHEEL_Data_t),DATA_WHEEL,1);
    
    dev_wheeldata_get(&wheeldata,2);
    pc_protocol_apendbuffer(sendbufff,&topcsize,sizeof(sendbufff),&wheeldata,sizeof(WHEEL_Data_t),DATA_WHEEL,2);
   
    dev_wheeldata_get(&wheeldata,3);
    pc_protocol_apendbuffer(sendbufff,&topcsize,sizeof(sendbufff),&wheeldata,sizeof(WHEEL_Data_t),DATA_WHEEL,3);
    driver_uart_send(HAL_UART_4, sendbufff, topcsize, 10, &erro);
    return 0;
}

