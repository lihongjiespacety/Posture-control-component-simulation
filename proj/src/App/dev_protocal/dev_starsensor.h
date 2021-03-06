#ifndef DEV_STARSENSOR_H
#define DEV_STARSENSOR_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#define STARSENSOR_NUM 3 /*定义星敏的个数*/
/**
 * \enum STARSENSOR_Data_t
 * 星敏子块内容.
 */
typedef __packed struct
{
    int32_t i;
    int32_t j;
    int32_t k;
    int32_t q;
    int32_t x_speed;
    int32_t y_speed;
    int32_t z_speed;
    uint8_t exposure;  /*曝光时长*/
    uint8_t state;    /*曝光时长*/
    int32_t sec;
    int32_t us;
}STARSENSOR_Data_t;


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
int32_t dev_starsensordata_handle(uint8_t* buff, uint8_t subtype, uint8_t size);


/**
 *******************************************************************************
 * \fn          int32_t dev_starsensordata_get(MAGTM_Data_t* buff,uint8_t subtype))
 * \brief       获取星敏数据.
 * \note        .
 * \param[in]   buff     存储获取到的数据.
 * \param[in]   subtype  0-n 多个设备指定是哪一个.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_starsensordata_get(STARSENSOR_Data_t* buff,uint8_t subtype);

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
int32_t dev_starsensortel_handle(uint8_t* buff, uint8_t subtype, uint8_t size);


#ifdef __cplusplus
}
#endif

#endif