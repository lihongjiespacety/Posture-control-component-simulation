#ifndef DEV_GPS_H
#define DEV_GPS_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#define GPS_NUM 1 /*定义GPS的个数*/
/**
 * \enum GPS_Data_t
 * GPS子块内容.
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
 * \brief       处理GPS子块数据.
 * \note        处理GPS子块数据.
 * \param[in]   buff     子块内容(不包括子块长度 子块类型 子块子类型).
 * \param[in]   subtype  子块子类型 一般多个同类型设备代表设备序号.
 * \param[in]   size     子块内容长度(不包括子块长度 子块类型 子块子类型).
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_gpsdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size);

/**
 *******************************************************************************
 * \fn          int32_t dev_gpsdata_get(GPS_Data_t* buff)
 * \brief       获取GPS数据.
 * \note        .
 * \param[in]   buff     存储获取到的数据.
 * \param[in]   subtype  0-n 多个设备指定是哪一个.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_gpsdata_get(GPS_Data_t* buff,uint8_t subtype);

/**
 *******************************************************************************
 * \fn          int32_t dev_gpstel_handle(uint8_t* buff, uint8_t size)
 * \brief       处理GPS遥测请求.
 * \note        处理GPS遥测请求.
 * \param[in]   buff     数据.
 * \param[in]   size     数据内容长度.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_gpstel_handle(uint8_t* buff, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif