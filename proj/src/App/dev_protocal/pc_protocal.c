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

#define READ_BUFF_MAX 256   /*��һ�δ��ڴ�С*/
#define READ_TIMEOUT 200   /*��һ�δ��ڳ�ʱʱ��*/

#define FRAME_MINI_SIZE 6   /*һ֡������С����*/
#define FRAME_HEAD_SIZE 5   /*֡ͷ����*/
#define FRAME_BUFF_MAX 512  /*֡����������С*/

#define HEAD_FLAG1 0xAA
#define HEAD_FLAG2 0x55


static uint8_t s_framebuff_au8[FRAME_BUFF_MAX];   /*֡��������*/
static uint32_t s_framebuff_size=0;              /*֡�����������ݳ���*/


/**
 *  ģ������֡                 ��ǿ��  10001nT 10002nT 10003nT              ����  (0i+0j+0k,1)   10000S  2mS                                                                                           ̫��  (0i+0j+0k,1)   10000S  2mS
 *  ����֡      AA 55 00 9D 00 0F 01 00 00 00 27 11 00 00 27 12 00 00 27 13 29 02 00 00 00 00 00 00 00 00 00 00 00 00 00 7F FF FF FF 00 00 27 11 00 00 00 31 00 00 00 00 00 00 00 00 00 00 00 00 00 00 1C 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
 *                             GPS                                                                        20��2��18�� 15ʱ43��01��900mS ����                                                          ����                                  У���
 *                             23 04 00 00 00 27 11 00 00 27 12 00 00 27 13 00 00 27 11 00 00 27 12 00 00 27 13 14 02 12 0F 2B 01 03 84 15 05 00 00 00 27 11 00 00 27 12 00 00 27 13 07 D0 07 D0 07 D0 0C 06 00 13 88 13 88 13 88 13 88 00  B5  
 *                             �������� 
 *
 * AA 55 00 9E 00 0F 01 00 00 00 27 11 00 00 27 12 00 00 27 13 29 02 00 00 00 00 00 00 00 00 00 00 00 00 00 7F FF FF FF 00 00 27 11 00 00 00 31 00 00 00 00 00 00 00 00 00 00 00 00 00 00 1C 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 23 04 00 00 00 27 11 00 00 27 12 00 00 27 13 00 00 27 11 00 00 27 12 00 00 27 13 14 02 12 0F 2B 01 03 84 15 05 00 00 00 27 11 00 00 27 12 00 00 27 13 07 D0 07 D0 07 D0 0C 06 00 13 88 13 88 13 88 13 88 00 B6
 *  ����֡     AA 55 xx xx 01
 */

/**
 *******************************************************************************
 * \fn          static int32_t pc_protocol_gethead(uint8_t* buff, uint32_t* index)
 * \brief       ��Ѱ֡ͷ��־.
 * \note        ����֡ͷ��־HEAD_FLAG1 HEAD_FLAG2.
 * \param[in]   buff     ָ��֡����������.
 * \param[in]   index    ָ����������.
 * \retval      >=0      ���ҵ�������λ��
 * \retval      <0       ����ʧ��
 *******************************************************************************
 */
static int32_t pc_protocol_gethead(uint8_t* buff, uint32_t* index)
{
    uint32_t i = 0;
    int32_t position = -1;
    if(*index < 2)
    {
        return -1;  /*С����С֡���� ֱ�ӷ���*/
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
 * \brief       PC������������֡����.
 * \note        ��������֡����.
 * \param[in]   buff     ָ���ӿ����ݲ�����֡��־ ���� ���� У��� buff���ܱ���д.
 * \param[in]   size     �ӿ����ݳ��Ȳ�����֡��־ ���� ���� У���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
static int32_t pc_protocol_handle_data(uint8_t* buff, uint32_t size)
{
    uint16_t datalen = 0;
    int32_t res = 0;   /*����״̬  1��Ҫ�������� 0����Ҫ��������ɹ� -1����Ҫ��������ʧ��*/
    do
    {
        datalen = buff[0];
        if(size > 0)
        {
            if(size >= datalen)
            {
                /*��Ч���ݳ��� ��������*/
                switch(buff[1])
                {
                    case DATA_MAGTM:
                        res = dev_magtmdata_handle(&buff[3], buff[2], datalen-3);  /*�����ӿ����ݺͳ���(�������ӿ鳤�� �ӿ����� �ӿ�������)*/
                    break;
                    case DATA_STARSENSOR:
                        res = dev_starsensordata_handle(&buff[3], buff[2], datalen-3);  /*�����ӿ����ݺͳ���(�������ӿ鳤�� �ӿ����� �ӿ�������)*/
                    break;
                    case DATA_SSOC:
                        res = dev_ssocdata_handle(&buff[3], buff[2], datalen-3);  /*�����ӿ����ݺͳ���(�������ӿ鳤�� �ӿ����� �ӿ�������)*/ 
                    break;
                    case DATA_GPS:
                        res = dev_gpsdata_handle(&buff[3], buff[2], datalen-3);  /*�����ӿ����ݺͳ���(�������ӿ鳤�� �ӿ����� �ӿ�������)*/
                    break;
                    case DATA_GYRO:
                        res = dev_gyrodata_handle(&buff[3], buff[2], datalen-3);  /*�����ӿ����ݺͳ���(�������ӿ鳤�� �ӿ����� �ӿ�������)*/
                    break;
                    case DATA_WHEEL:
                        res = dev_wheeldata_handle(&buff[3], buff[2], datalen-3);  /*�����ӿ����ݺͳ���(�������ӿ鳤�� �ӿ����� �ӿ�������)*/
                    break;
                    default:
                        res = -1;
                    break;
                
                }
                /*�������ݼ�������*/
                if(res == 0)
                {
                    memmove(buff,buff+datalen,size - datalen);
                    size -= datalen;
                    res = 1;
                }
                else
                {
                    /*����ʧ��*/
                    res = -1;
                }
            }
            else
            {
                /*��Ч���ݳ���*/
                res = -1;
            }
        }
        else
        {
            res = 0;   /*���ݴ�����*/
        }
    }while(res == 1);  /*�����Ҫ���������ͷ��ʼ*/

    return res;
}

/**
 *******************************************************************************
 * \fn          static int32_t pc_protocol_handle_cmd(uint8_t* buff, uint32_t size)
 * \brief       PC������������֡����.
 * \note        ��������֡����.
 * \param[in]   buff     ָ���ӿ����ݲ�����֡��־ ���� ���� У��� buff���ܱ���д.
 * \param[in]   size     �ӿ����ݳ��Ȳ�����֡��־ ���� ���� У���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
static int32_t pc_protocol_handle_cmd(uint8_t* buff, uint32_t size)
{
    uint16_t datalen = 0;
    int32_t res = 0;   /*����״̬  1��Ҫ�������� 0����Ҫ��������ɹ� -1����Ҫ��������ʧ��*/
    do
    {
        datalen = buff[0];
        if(size > 0)
        {
            if(size >= datalen)
            {
                /*��Ч���ݳ��� ��������*/
                switch(buff[1])
                {
                    case 0:
                    break;
                    case 1:
                    break;
                
                }
                /*�������ݼ�������*/
                memmove(buff,buff+datalen,size - datalen);
                size -= datalen;
                res = 1;
            }
            else
            {
                /*��Ч���ݳ���*/
                res = -1;
            }
        }
        else
        {
            res = 0;   /*���ݴ�����*/
        }
    }while(res == 1);  /*�����Ҫ���������ͷ��ʼ*/

    return res;
}



/**
 *******************************************************************************
 * \fn          static int32_t pc_protocol_handle(uint8_t* buff, uint32_t* size)
 * \brief       PC��������֡���ݴ���.
 * \note        ���յ��������ݺ���øú�������.
 * \param[in]   buff     ָ��֡����������,���ݿ��ܱ���д.
 * \param[in]   size     ֡���ݳ���,���ݿ��ܻᱻ��д.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
static int32_t pc_protocol_handle(uint8_t* buff, uint32_t* size)
{
    int32_t headposition = -1;
    int32_t res = 0;   /*����״̬  1��Ҫ�������� 0����Ҫ��������ɹ� -1����Ҫ��������ʧ��*/
    uint16_t datalen = 0;
    if(*size <= 6)
    {
        return 0;  /*С����С֡���� ֱ�ӷ���*/
    }else{}
    
    do
    {
        headposition = pc_protocol_gethead(buff,size);
        if(headposition >= 0)
        {
            /*����ɾ��ǰ����Ч������*/
            if(headposition > 0)
            {
                memmove(buff,buff+headposition,*size- headposition);
                *size -= headposition;
            }
            else
            {
                /*headposition == 0����Ҫ����*/
            }
            
            if(*size > FRAME_MINI_SIZE)
            {
                datalen = ((uint16_t)buff[2]<<8) | (uint16_t)buff[3];  /*����*/
                if((datalen<=FRAME_BUFF_MAX) && (datalen>=FRAME_MINI_SIZE))
                {
                    if(*size >= datalen)
                    {
                        /*��������ȫ*/
                        if(buffer_checksum(buff, datalen-1) == buff[datalen-1])
                        {
                             /*����У��ɹ�  ��������*/
                            OsPrintf(OSAPI_DEBUG_INFO,"get frame type=%d framelen=%d\r\n", buff[4],datalen);
                            switch(buff[4])
                            {
                                case 0:
                                    pc_protocol_handle_data(buff+FRAME_HEAD_SIZE,datalen-FRAME_MINI_SIZE);  /*��������֡*/
                                    pc_protocol_ackhandle(); /*���ͷ���֡����*/
                                break;
                                case 1:
                                    pc_protocol_handle_cmd(buff+FRAME_HEAD_SIZE,datalen-FRAME_MINI_SIZE);  /*��������֡*/
                                break;
                                default:
                                break;
                            }
                            memmove(buff,buff+datalen,*size - datalen);  /*ɾ�������Ѿ��������֡����*/
                            *size -= datalen;
                            res = 1;                            /*��Ҫ��������*/
                        }
                        else
                        {
                            /*����У�����  ɾ��֡ͷ��������*/
                            memmove(buff,buff+2,*size - 2);   
                            *size -= 2;
                            res = 1;  /*��Ҫ��������*/
                        }
                    }
                    else
                    {
                        res = -1;  /*����δ��ȫ ���账��*/   
                    }
                
                }
                else
                {
                    /*���ȴ���  ɾ��֡ͷ��������*/
                    memmove(buff,buff+2,*size - 2);   
                    *size -= 2;
                    res = 1;  /*��Ҫ��������*/
                }
            }
            else
            {
                res = -1;  /*����̫�����账��*/
            }
        }
        else
        {
            /*û���ҵ���־  ��д����ֵ ʹ������������Ч ��ͷ��ʼ*/
            *size = 0;
            res = -1;
        }
        
    }while(res == 1);  /*�����Ҫ���������ͷ��ʼ*/
    return res;
}



/**
 * ��PCͨѶ����Э�鴦������
 */
void uarttask(void *pvParameters )
{
    uint32_t readlen = 0;                   /*����ʵ�ʶ������ֽ���*/
    uint32_t toreadlen = 0;                 /*������Ҫ�����ֽ���*/
    uint8_t readbuff[READ_BUFF_MAX] = {0};  /*���εĶ�����*/
    int32_t erro = 0;                       /*��������*/
    /*��ʼ����PCͨѶ�Ĵ���*/
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
        /*�����յ����ݽ���ת��*/
        do
        {
            if((FRAME_BUFF_MAX-s_framebuff_size)>=READ_BUFF_MAX)
            {
                toreadlen = READ_BUFF_MAX;  /*���֡�������ܷŵ������ݴ��� �������� ��ôһ�ζ���������С*/
            }
            else
            {
                toreadlen = FRAME_BUFF_MAX-s_framebuff_size;  /*���֡�������ܷŵ�������С�� �������� ��ôֻ���ܴ���µĴ�С*/
            }
            readlen = driver_uart_recv(HAL_UART_4, readbuff, toreadlen, READ_TIMEOUT, &erro);
            if(readlen > 0)
            {
                memcpy(&s_framebuff_au8[s_framebuff_size],readbuff,readlen);
                s_framebuff_size += readlen;
            }
            else
            {
                /*δ�������ݼ�����*/
            }
            /*����֡����ӿ�*/
            pc_protocol_handle(s_framebuff_au8, &s_framebuff_size);
        }
        while(readlen);
        // OsTimeDelay(10);   // driver_uart_recv��������ʱ���� ����Ҫ�ٵ���.
    }
}

/**
 *******************************************************************************
 * \fn          int32_t pc_protocol_initbuffer(uint8_t* buff, uint32_t* size,uint32_t bufflen)
 * \brief       ���һ����pcͨѶ��ʼ���Ŀհ�.
 * \note        .
 * \param[in]   buff     ָ��֡����������,���ݿ��ܱ���д �����߷���.
 * \param[in]   size     ��Ч֡���ݳ���,���ݿ��ܻᱻ��д.
 * \param[in]   bufflen  ��Ч֡���ݳ���,���ݿ��ܻᱻ��д ������ָ��.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
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
    buff[4] = 0x02;  /*�������ݰ�*/
    buff[5] = 0x02;
    *size = 6;
    return 0;
}

/**
 *******************************************************************************
 * \fn          int32_t pc_protocol_apendbuffer(uint8_t* buff, uint32_t* size,uint32_t bufflen,void* adddata,uint32_t adddatalen,uint8_t type, uint8_t subtype)
 * \brief       ���һ�����ݿ�.
 * \note        .
 * \param[in]   buff     ָ��֡����������,���ݿ��ܱ���д �����߷���.
 * \param[in]   size     ��Ч֡���ݳ���,���ݿ��ܻᱻ��д.
 * \param[in]   adddata  ����ӵ�����.
 * \param[in]   adddatalen  �������������.
 * \param[in]   type  ����.
 * \param[in]   subtype  ������.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
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
    //buff[4] = 0x02;  /*�������ݰ�*/
    
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
 * \brief       PC��������֡���ݻ�Ӧ����(�������ݰ�).
 * \note        ���յ��������ݺ���øú�������.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
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

