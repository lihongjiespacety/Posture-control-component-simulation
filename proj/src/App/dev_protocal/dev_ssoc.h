#ifndef DEV_SSOC_H
#define DEV_SSOC_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#define SSOC_NUM 1 /*定义太敏的个数*/
/**
 * \enum SSOC_Data_t
 * 太敏子块内容.
 */
typedef __packed struct
{
    float  erfa;
    float  beta;
    uint8_t sta;
    float  cell1;
    float  cell2;
    float  cell3;
    float  cell4;
}SSOC_Data_t;


/**
 *******************************************************************************
 * \fn          int32_t dev_ssocdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       处理太敏子块数据.
 * \note        处理太敏子块数据.
 * \param[in]   buff     子块内容(不包括子块长度 子块类型 子块子类型).
 * \param[in]   subtype  子块子类型 一般多个同类型设备代表设备序号.
 * \param[in]   size     子块内容长度(不包括子块长度 子块类型 子块子类型).
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_ssocdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size);


/**
 *******************************************************************************
 * \fn          int32_t dev_ssocdata_get(SSOC_Data_t* buff,uint8_t subtype))
 * \brief       获取太敏数据.
 * \note        .
 * \param[in]   buff     存储获取到的数据.
 * \param[in]   subtype  0-n 多个设备指定是哪一个.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_ssocdata_get(SSOC_Data_t* buff,uint8_t subtype);

/**
 *******************************************************************************
 * \fn          int32_t dev_ssoctel_handle(uint8_t* buff, uint8_t size)
 * \brief       处理太敏遥测请求.
 * \note        处理太敏遥测请求.
 * \param[in]   buff     数据.
 * \param[in]   size     数据内容长度.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_ssoctel_handle(uint8_t* buff, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif