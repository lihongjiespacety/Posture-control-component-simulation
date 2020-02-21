#ifndef DEV_GPS_H
#define DEV_GPS_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#define GPS_NUM 1 /*����GPS�ĸ���*/
/**
 * \enum GPS_Data_t
 * GPS�ӿ�����.
 */
typedef __packed struct
{
    uint32_t x_position;
    uint32_t y_position;
    uint32_t z_position;
    uint32_t x_speed;
    uint32_t y_speed;
    uint32_t z_speed; 
    uint8_t  year; 
    uint8_t  month;
    uint8_t  day; 
    uint8_t  hour; 
    uint8_t  min;
    uint8_t  sec; 
    uint16_t msec; 
}GPS_Data_t;


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
int32_t dev_gpsdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size);

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
int32_t dev_gpsdata_get(GPS_Data_t* buff,uint8_t subtype);

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
int32_t dev_gpstel_handle(uint8_t* buff, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif