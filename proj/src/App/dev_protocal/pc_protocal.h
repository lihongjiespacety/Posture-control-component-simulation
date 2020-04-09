#ifndef PC_PROTOCAL_H
#define PC_PROTOCAL_H

#ifdef __cplusplus
 extern "C" {
#endif
   
void uarttask(void *pvParameters );

/**
 * \enum DATA_TYPE_e
 * 数据帧子块类型.
 */
typedef enum
{
    DATA_MAGTM = 1,
    DATA_STARSENSOR = 2,
    DATA_SSOC = 3,
    DATA_GPS = 4,
    DATA_GYRO = 5,
    DATA_WHEEL = 6,
    DATA_MAGT = 7,
}DATA_TYPE_e;

/**
 * \enum CMD_TYPE_e
 * 命令帧子块类型.
 */
typedef enum
{
    CMD_TEST = 0,
}CMD_TYPE_e;


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
int32_t pc_protocol_initbuffer(uint8_t* buff, uint32_t* size,uint32_t bufflen);

   
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
int32_t pc_protocol_apendbuffer(uint8_t* buff, uint32_t* size,uint32_t bufflen,void* adddata,uint32_t adddatalen,uint8_t type, uint8_t subtype);


/**
 *******************************************************************************
 * \fn          int32_t pc_protocol_ackhandle(void)
 * \brief       PC发过来的帧数据回应处理(返回数据包).
 * \note        接收到串口数据后调用该函数处理.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t pc_protocol_ackhandle(void);

/**
 *******************************************************************************
 * \fn          uint8_t get_dev_state(void)
 * \brief       获取设备静默状态.
 * \note        .
 * \return      uint8_t 状态值
 *******************************************************************************
 */
uint8_t get_dev_state(void);


#ifdef __cplusplus
}
#endif

#endif