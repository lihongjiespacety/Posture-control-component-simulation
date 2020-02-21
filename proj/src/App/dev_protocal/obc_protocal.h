#ifndef OBC_PROTOCAL_H
#define OBC_PROTOCAL_H

#ifdef __cplusplus
 extern "C" {
#endif
   

/**
 *******************************************************************************
 * \fn          int32_t obc_protocol_handle_data(uint8_t canid, uint8_t* buff, uint32_t size)
 * \brief       OBC发过来的数据帧处理.
 * \note        处理数据帧内容.
 * \param[in]   canid    模拟的can设备的id.
 * \param[in]   buff     指向数据内容.
 * \param[in]   size     数据的长度.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t obc_protocol_handle_data(uint8_t canid, uint8_t* buff, uint32_t size);


#ifdef __cplusplus
}
#endif

#endif