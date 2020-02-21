#ifndef COMMON_H
#define COMMON_H

#ifdef __cplusplus
 extern "C" {
#endif

/**
 *******************************************************************************
 * \fn          int32_t buffer_get_int32(uint8_t* buff)
 * \brief       从指定缓冲区组合一个int32_t数据,缓冲区数据按大端模式.
 * \note        .
 * \param[in]   buff     缓冲区 至少有4字节有效数据.
 * \return      int32_t
 *******************************************************************************
 */
int32_t buffer_get_int32(uint8_t* buff);

/**
 *******************************************************************************
 * \fn          uint32_t buffer_get_uint32(uint8_t* buff)
 * \brief       从指定缓冲区组合一个uint32_t数据,缓冲区数据按大端模式.
 * \note        .
 * \param[in]   buff     缓冲区 至少有4字节有效数据.
 * \return      uint32_t
 *******************************************************************************
 */
uint32_t buffer_get_uint32(uint8_t* buff);

/**
 *******************************************************************************
 * \fn          int16_t buffer_get_int16(uint8_t* buff)
 * \brief       从指定缓冲区组合一个int16_t数据,缓冲区数据按大端模式.
 * \note        .
 * \param[in]   buff     缓冲区 至少有2字节有效数据.
 * \return      int16_t
 *******************************************************************************
 */
int16_t buffer_get_int16(uint8_t* buff);

/**
 *******************************************************************************
 * \fn          uint16_t buffer_get_uint16(uint8_t* buff)
 * \brief       从指定缓冲区组合一个uint16_t数据,缓冲区数据按大端模式.
 * \note        .
 * \param[in]   buff     缓冲区 至少有2字节有效数据.
 * \return      uint16_t
 *******************************************************************************
 */
uint16_t buffer_get_uint16(uint8_t* buff);

/**
 *******************************************************************************
 * \fn          float buffer_get_float(uint8_t* buff)
 * \brief       从指定缓冲区组合一个float数据,缓冲区数据按大端模式.
 * \note        .
 * \param[in]   buff     缓冲区 至少有4字节有效数据.
 * \return      float
 *******************************************************************************
 */
float buffer_get_float(uint8_t* buff);

/**
 *******************************************************************************
 * \fn          void buffer_set_int16(uint8_t* buff,int16_t val)
 * \brief       将int16_t 写入指定缓冲区大端模式.
 * \note        .
 * \param[out]   buff     缓冲区 至少有2字节有效数据.
 * \param[in]    val      待转换的值.
 * \return       void
 *******************************************************************************
 */
void buffer_set_int16(uint8_t* buff,int16_t val);

/**
 *******************************************************************************
 * \fn          void buffer_set_uint16(uint8_t* buff,uint16_t val)
 * \brief       将uint16_t 写入指定缓冲区大端模式.
 * \note        .
 * \param[out]   buff     缓冲区 至少有2字节有效数据.
 * \param[in]    val      待转换的值.
 * \return       void
 *******************************************************************************
 */
void buffer_set_uint16(uint8_t* buff,uint16_t val);


/**
 *******************************************************************************
 * \fn          void buffer_set_int32(uint8_t* buff,int32_t val)
 * \brief       将int32_t 写入指定缓冲区大端模式.
 * \note        .
 * \param[out]   buff     缓冲区 至少有4字节有效数据.
 * \param[in]    val      待转换的值.
 * \return       void
 *******************************************************************************
 */
void buffer_set_int32(uint8_t* buff,int32_t val);

/**
 *******************************************************************************
 * \fn          void buffer_set_uint32(uint8_t* buff,int32_t val)
 * \brief       将uint32_t 写入指定缓冲区大端模式.
 * \note        .
 * \param[out]   buff     缓冲区 至少有4字节有效数据.
 * \param[in]    val      待转换的值.
 * \return       void
 *******************************************************************************
 */
void buffer_set_uint32(uint8_t* buff,uint32_t val);

/**
 *******************************************************************************
 * \fn          void buffer_set_float(uint8_t* buff,float val)
 * \brief       将float 写入指定缓冲区大端模式.
 * \note        .
 * \param[out]   buff     缓冲区 至少有4字节有效数据.
 * \param[in]    val      待转换的值.
 * \return       void
 *******************************************************************************
 */
void buffer_set_float(uint8_t* buff,float val);

/**
 *******************************************************************************
 * \fn          void buffer_swap(uint8_t* buff,uint8_t len)
 * \brief       buff指定长度的数据交换大小端.
 * \note        .
 * \param[out]   buff     缓冲区 .
 * \param[in]    val      待转换的长度.
 * \return       void
 *******************************************************************************
 */
void buffer_swap(uint8_t* buff,uint8_t len);

/**
 *******************************************************************************
 * \fn          uint8_t buffer_checksum(uint8_t* buff, uint32_t len)
 * \brief       计算累加和.
 * \note        计算8位累加和.
 * \param[in]   buff     指向数据.
 * \param[in]   len      数据长度.
 * \return      uint8_t   校验值
 *******************************************************************************
 */
uint8_t buffer_checksum(uint8_t* buff, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif