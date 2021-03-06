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
 * \enum DEV_NUM_e
 * 设备编号枚举.
 */
typedef enum
{
    DEV_NUM_MAGTM1 = 0,
    DEV_NUM_MAGTM2,
    DEV_NUM_MAGTM3,
    DEV_NUM_MAGTM4,
    DEV_NUM_STARSENSOR1,
    DEV_NUM_STARSENSOR2,
    DEV_NUM_STARSENSOR3,
    DEV_NUM_STARSENSOR4,
    DEV_NUM_SSOC1,
    DEV_NUM_SSOC2,
    DEV_NUM_SSOC3,
    DEV_NUM_SSOC4,
    DEV_NUM_GPS1,
    DEV_NUM_GPS2,
    DEV_NUM_GPS3,
    DEV_NUM_GPS4,
    DEV_NUM_GYRO1,
    DEV_NUM_GYRO2,
    DEV_NUM_GYRO3,
    DEV_NUM_GYRO4,
    DEV_NUM_WHEEL1,
    DEV_NUM_WHEEL2,
    DEV_NUM_WHEEL3,
    DEV_NUM_WHEEL4,
    DEV_NUM_MAGT1,
    DEV_NUM_MAGT2,
    DEV_NUM_MAGT3,
    DEV_NUM_MAGT4,
}DEV_NUM_e;

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
 * \fn          int32_t pc_protocol_setswitch(uint8_t* buff)
 * \brief       设置继电器参数.
 * \note        
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t pc_protocol_setswitch(uint8_t* buff);

/**
 *******************************************************************************
 * \fn          int32_t pc_protocol_ackswitch(uint8_t ch)
 * \brief       返回继电器参数.
 * \note        接收到读取参数指令后返回.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t pc_protocol_ackswitch(uint8_t ch);

/**
 *******************************************************************************
 * \fn          uint32_t get_dev_state(void)
 * \brief       获取设备静默状态.
 * \note        .
 * \return      uint32_t 状态值
 *******************************************************************************
 */
uint32_t get_dev_state(void);

void switch_set_par(uint8_t ch,uint32_t onmin,uint32_t onmax,uint32_t offmin,uint32_t offmax);

void switch_get_par(uint8_t ch,uint32_t* onmin,uint32_t* onmax,uint32_t* offmin,uint32_t* offmax);
#ifdef __cplusplus
}
#endif

#endif