#include <stdint.h>
#include <string.h>
#include <time.h>
#include "common.h"
#include "osapi_freertos.h"
#include "dev_gps.h"
#include "can.h"
#include "clock.h"
#include "driver_time.h"
#include "obc_protocal.h"
#include "pc_protocal.h"

GPS_Data_t s_gps_data_at[GPS_NUM];  /*��¼PC������������*/

static uint8_t Num_C = 0;    /*������ȷ֡����*/
static uint8_t Num_T = 0;    /*ң����ȷ֡����*/
static uint8_t LCmd_ID = 0;  /*���ִ��ָ����*/
static uint8_t Num_RC = 0;  /*����֡����*/
static uint8_t Can_id = GPS_CANID;  /*ԭʼ���ݰ�CANID*/
static uint8_t Gps_State = 1;  /*��������״̬ 0 ������ 1����*/

/**
 *******************************************************************************
 * \fn          int32_t dev_gpsdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       ����GPS�ӿ�����.
 * \note        ����GPS�ӿ�����.
 * \param[in]   buff     �ӿ�����(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \param[in]   subtype  �ӿ������� һ����ͬ�����豸�����豸���.
 * \param[in]   size     �ӿ����ݳ���(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_gpsdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
{
    timestamp_t ts;
    struct tm t_st;
    time_t t;
    if((buff == 0) || (subtype>=GPS_NUM))
    {
        return -1;
    }
    else
    {
        if(size == sizeof(GPS_Data_t))
        {
            __disable_interrupt();      /*��֤������������ͬʱ���µ�*/  
            s_gps_data_at[subtype].x_position  = buffer_get_int32(&buff[0]);
            s_gps_data_at[subtype].y_position  = buffer_get_int32(&buff[4]);
            s_gps_data_at[subtype].z_position  = buffer_get_int32(&buff[8]);
            s_gps_data_at[subtype].x_speed  = buffer_get_int32(&buff[12]);
            s_gps_data_at[subtype].y_speed  = buffer_get_int32(&buff[16]);
            s_gps_data_at[subtype].z_speed  = buffer_get_int32(&buff[20]);
            s_gps_data_at[subtype].year  = buff[24];
            s_gps_data_at[subtype].month = buff[25];
            s_gps_data_at[subtype].day  =  buff[26];
            s_gps_data_at[subtype].hour =  buff[27];
            s_gps_data_at[subtype].min =   buff[28];
            s_gps_data_at[subtype].sec  =  buff[29];
            s_gps_data_at[subtype].msec  = buffer_get_uint16(&buff[30]);
            __enable_interrupt(); 
    
            t_st.tm_year = s_gps_data_at[subtype].year + 100;  /* Year - 1900. */
            t_st.tm_mon = s_gps_data_at[subtype].month - 1;    /* Month. [0-11] */
            t_st.tm_mday = s_gps_data_at[subtype].day;         /* Day. [1-31] */
            t_st.tm_hour = s_gps_data_at[subtype].hour;        /* Hours. [0-23] */
            t_st.tm_min = s_gps_data_at[subtype].min;          /* Minutes. [0-59] */
            t_st.tm_sec = s_gps_data_at[subtype].sec;          /* Seconds. [0-60] (1 leap second) */
            t_st.tm_isdst = -1;                                /* DST. [-1/0/1]*/
            //t_st.tm_wday;                                    /* Day of week. [0-6] */
            //t_st.tm_yday;                                    /* Days in year.[0-365] */
            t = driver_time_mktime_r(&t_st);
            ts.tv_sec = t;
            ts.tv_usec = s_gps_data_at[subtype].msec*1000;
            clock_set_time(&ts);
            OsPrintf(OSAPI_DEBUG_INFO,"get gpsdata\r\n");
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
 * \fn          int32_t dev_gpsdata_get(GPS_Data_t* buff)
 * \brief       ��ȡGPS����.
 * \note        .
 * \param[in]   buff     �洢��ȡ��������.
 * \param[in]   subtype  0-n ����豸ָ������һ��.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_gpsdata_get(GPS_Data_t* buff,uint8_t subtype)
{
    if((buff == 0) || (subtype>=GPS_NUM))
    {
        return -1;
    }
    else
    {
        memcpy(buff,&s_gps_data_at[subtype],sizeof(GPS_Data_t));
        return 0;
    }
}

/**
 *******************************************************************************
 * \fn          int32_t dev_gpstel_handle(uint8_t* buff, uint8_t size)
 * \brief       ����GPSң������.
 * \note        ����GPSң������.
 * \param[in]   buff     ����.
 * \param[in]   size     �������ݳ���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_gpstel_handle(uint8_t* buff, uint8_t size)
{
    uint8_t sendbufff[465] = {0};
    timestamp_t ts;
    struct tm t_st;
    
    /*���յ�ң������ת����PC*/
  
    int32_t x_speed;
    int32_t y_speed;
    int32_t z_speed;
    int32_t x_position;
    int32_t y_position;
    int32_t z_position;
    uint8_t y;
    uint8_t m;
    uint8_t d;
    uint8_t h;
    uint8_t min;
    uint8_t sec;
    uint16_t ms;
    clock_get_time(&ts);
    
    driver_time_localtime_r(&ts.tv_sec, &t_st);
    y = t_st.tm_year-100;
    m = t_st.tm_mon +1;
    d = t_st.tm_mday;
    h = t_st.tm_hour;
    min = t_st.tm_min;
    sec = t_st.tm_sec;
    ms = (ts.tv_usec+500)/1000;
    
    __disable_interrupt();      /*��֤������������ͬʱ���µ�*/    
    x_speed =  s_gps_data_at[0].x_speed;
    y_speed =  s_gps_data_at[0].y_speed;
    z_speed =  s_gps_data_at[0].z_speed;
    x_position =  s_gps_data_at[0].x_position;
    y_position =  s_gps_data_at[0].y_position;
    z_position =  s_gps_data_at[0].z_position;
    __enable_interrupt(); 
    
    
    /*���յ�ң��������Ӧң���
     *   PVTң������  00 01 55 55 55 55 55 55
     *   ԭʼ����ң������ 00 02 AA AA AA AA AA AA
     *   �����ֹ 05 01 33 33 33 33 33 33
     *   �������� 05 02 55 55 55 55 55 55
     *   ԭʼ���ݰ�CANID���� 05 03 xx 55 55 55 55 55
     */
    if(size == 8)
    {
        if((buff[0]==0x05) && (buff[1]==0x03))
        {
            Num_C++;
            Can_id = buff[2];
            LCmd_ID = 5;
            sendbufff[0] = 0;
            sendbufff[1] = 6;
            sendbufff[2] = 0x35;
            sendbufff[3] = 0x03;
            sendbufff[4] = 0x55;
            sendbufff[5] = 0x55;
            sendbufff[6] = 0x55;
            sendbufff[7] = 0x55;
            if((get_dev_state() & (1<< DATA_GPS))== 0)
            {
            can_tx_raw_data(GPS_CANID,GOM_OBC_CANID,sendbufff,8,CFP_SINGLE,1,10);
        }
        }
        if((buff[0]==0x05) && (buff[1]==0x02))
        {
            Num_C++;
            Gps_State = 1;
            LCmd_ID = 5;
            sendbufff[0] = 0;
            sendbufff[1] = 6;
            sendbufff[2] = 0x35;
            sendbufff[3] = 0x03;
            sendbufff[4] = 0x55;
            sendbufff[5] = 0x55;
            sendbufff[6] = 0x55;
            sendbufff[7] = 0x55;
            if((get_dev_state() & (1<< DATA_GPS))== 0)
            {
            can_tx_raw_data(GPS_CANID,GOM_OBC_CANID,sendbufff,8,CFP_SINGLE,1,10);
        }
        }
        if((buff[0]==0x05) && (buff[1]==0x01))
        {
            Num_C++;
            Gps_State = 0;
            LCmd_ID = 5;
            sendbufff[0] = 0;
            sendbufff[1] = 6;
            sendbufff[2] = 0x35;
            sendbufff[3] = 0x03;
            sendbufff[4] = 0x55;
            sendbufff[5] = 0x55;
            sendbufff[6] = 0x55;
            sendbufff[7] = 0x55;
            if((get_dev_state() & (1<< DATA_GPS))== 0)
            {
            can_tx_raw_data(GPS_CANID,GOM_OBC_CANID,sendbufff,8,CFP_SINGLE,1,10);
        }
        }
        if((buff[0]==0x00) && (buff[1]==0x02))
        {
            Num_T++;
            Num_C++;
            LCmd_ID = 0;
            sendbufff[0] = 0x01;
            sendbufff[1] = 0xCE;
            sendbufff[2] = 0x35;
            sendbufff[3] = 0x02;
            sendbufff[4] = Num_T;
            sendbufff[5] = Num_C;
            sendbufff[6] = Num_RC;
            sendbufff[7] = LCmd_ID;
            sendbufff[464] = buffer_checksum(sendbufff,464);
            if((get_dev_state() & (1<< DATA_GPS))== 0)
            {
            can_tx_raw_data(Can_id,GOM_OBC_CANID,sendbufff,465,CFP_BEGIN,1,100);
        }
        }
        if((buff[0]==0x00) && (buff[1]==0x01))
        {
            Num_T++;
            LCmd_ID = 0;
            Num_C++;
            sendbufff[0] = 0x00;
            sendbufff[1] = 0x54;
            sendbufff[2] = 0x35;
            sendbufff[3] = 0x01;
            sendbufff[4] = Num_T;
            sendbufff[5] = Num_C;
            sendbufff[6] = Num_RC;
            sendbufff[7] = LCmd_ID;
            
            sendbufff[8] = d;
            sendbufff[9] = m;
            sendbufff[10] = y;
            sendbufff[11] = h;
            sendbufff[12] = min;
            sendbufff[13] = sec;
           // buffer_set_uint16(&sendbufff[14],ms);
            sendbufff[14] = ms & 0xFF;  /*С��*/
            sendbufff[15] = ms >> 8;
            
            sendbufff[16] = 0xAA;
            buffer_set_uint32(&sendbufff[24],x_position);
            buffer_set_uint32(&sendbufff[28],y_position);
            buffer_set_uint32(&sendbufff[32],z_position);
            buffer_set_uint32(&sendbufff[36],x_speed);
            buffer_set_uint32(&sendbufff[40],y_speed);
            buffer_set_uint32(&sendbufff[44],z_speed);
            
            buffer_set_uint32(&sendbufff[48],x_position);
            buffer_set_uint32(&sendbufff[52],y_position);
            buffer_set_uint32(&sendbufff[56],z_position);
            buffer_set_uint32(&sendbufff[60],x_speed);
            buffer_set_uint32(&sendbufff[64],y_speed);
            buffer_set_uint32(&sendbufff[68],z_speed);
            
            /*��������״̬*/
            if(Gps_State)
            {
                
                sendbufff[81] &= ~(1<<4);
                sendbufff[81] |= 0x82;
            }
            else
            {
                sendbufff[81] |= (1<<4);
            }
            
            sendbufff[86] = buffer_checksum(sendbufff,86);
            if((get_dev_state() & (1<< DATA_GPS))== 0)
            {
            can_tx_raw_data(Can_id,GOM_OBC_CANID,sendbufff,87,CFP_BEGIN,1,100);
        }
        }
        else
        {
            Num_RC++;
        }
    }
    else
    {
        Num_RC++;
    }

    return 0;
}

