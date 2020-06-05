#ifndef DEV_WHEEL_H
#define DEV_WHEEL_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#define WHEEL_NUM 4 /*定义飞轮的个数*/
/**
 * \enum WHEEL_Data_t
 * 飞轮子块内容.
 */
typedef __packed struct
{
    float   speed;
    float   toq;
    uint8_t mode;
    float   rspeed;
}WHEEL_Data_t;


/**
 * \enum WHEEL_Type_e
 * 飞轮类型枚举.
 */
typedef enum  
{
    WHEEL_TYPE_VRW = 0,   /*德国飞轮*/
    WHEEL_TYPE_LY,        /*揽月飞轮*/
}WHEEL_Type_e;



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
int32_t dev_wheeldata_handle(uint8_t* buff, uint8_t subtype, uint8_t size);


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
int32_t dev_wheeldata_get(WHEEL_Data_t* buff,uint8_t subtype);

/**
 *******************************************************************************
 * \fn          int32_t dev_wheelxtel_handle(uint8_t* buff, uint8_t size)
 * \brief       处理飞轮遥测请求.
 * \note        处理飞轮遥测请求.
 * \param[in]   buff     数据.
 * \param[in]   size     数据内容长度.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_wheelxtel_handle(uint8_t* buff, uint8_t size);

/**
 *******************************************************************************
 * \fn          int32_t dev_wheelytel_handle(uint8_t* buff, uint8_t size)
 * \brief       处理飞轮遥测请求.
 * \note        处理飞轮遥测请求.
 * \param[in]   buff     数据.
 * \param[in]   size     数据内容长度.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_wheelytel_handle(uint8_t* buff, uint8_t size);

/**
 *******************************************************************************
 * \fn          int32_t dev_wheelztel_handle(uint8_t* buff, uint8_t size)
 * \brief       处理飞轮遥测请求.
 * \note        处理飞轮遥测请求.
 * \param[in]   buff     数据.
 * \param[in]   size     数据内容长度.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_wheelztel_handle(uint8_t* buff, uint8_t size);

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
int32_t dev_wheeltel_handle(uint8_t* buff, uint8_t subtype, uint8_t size);

/**
 *******************************************************************************
 * \fn          int32_t dev_wheeltel_tlhandle(uint8_t* buff, uint8_t canid, uint8_t size)
 * \brief       处理飞轮遥测包.
 * \note        处理飞轮遥测包.
 * \param[in]   buff     数据.
 * \param[in]   canid  id.
 * \param[in]   size     数据内容长度.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_wheeltel_tlhandle(uint8_t* buff, uint8_t canid, uint8_t size);

void dev_set_wheeltype(uint8_t type);

#ifdef __cplusplus
}
#endif

#endif