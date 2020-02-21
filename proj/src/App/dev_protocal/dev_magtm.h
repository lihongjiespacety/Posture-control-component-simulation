#ifndef DEV_MAGTM_H
#define DEV_MAGTM_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#define MAGTM_NUM 1 /*定义磁强计的个数*/
/**
 * \enum MAGTM_Data_t
 * 磁强计子块内容.
 */
typedef __packed struct
{
    int32_t mx;
    int32_t my;
    int32_t mz;
}MAGTM_Data_t;

/**
 * \struct zju_magm_ptm_t
 * 浙大磁强计遥测数据结构.
 */
typedef struct zju_magm_ptm 
{
	int32_t x_i32;      /**<  nT    */
	int32_t y_i32;      /**<  nT    */
	int32_t z_i32;      /**<  nT    */
} zju_magm_ptm_t;


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
int32_t dev_magtmdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size);


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
int32_t dev_magtmdata_get(MAGTM_Data_t* buff,uint8_t subtype);

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
int32_t dev_magtmtel_handle(uint8_t* buff, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif