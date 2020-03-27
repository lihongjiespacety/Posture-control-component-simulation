#include <stdint.h>

/**
 *******************************************************************************
 * \fn          int32_t buffer_get_int32(uint8_t* buff)
 * \brief       ��ָ�����������һ��int32_t����,���������ݰ����ģʽ.
 * \note        .
 * \param[in]   buff     ������ ������4�ֽ���Ч����.
 * \return      int32_t
 *******************************************************************************
 */
int32_t buffer_get_int32(uint8_t* buff)
{
    return ((uint32_t)buff[0]<<24) | ((uint32_t)buff[1]<<16) | ((uint32_t)buff[2]<<8) | ((uint32_t)buff[3]<<0);
}

/**
 *******************************************************************************
 * \fn          uint32_t buffer_get_uint32(uint8_t* buff)
 * \brief       ��ָ�����������һ��uint32_t����,���������ݰ����ģʽ.
 * \note        .
 * \param[in]   buff     ������ ������4�ֽ���Ч����.
 * \return      uint32_t
 *******************************************************************************
 */
uint32_t buffer_get_uint32(uint8_t* buff)
{
    return ((uint32_t)buff[0]<<24) | ((uint32_t)buff[1]<<16) | ((uint32_t)buff[2]<<8) | ((uint32_t)buff[3]<<0);
}

/**
 *******************************************************************************
 * \fn          int16_t buffer_get_int16(uint8_t* buff)
 * \brief       ��ָ�����������һ��int16_t����,���������ݰ����ģʽ.
 * \note        .
 * \param[in]   buff     ������ ������2�ֽ���Ч����.
 * \return      int16_t
 *******************************************************************************
 */
int16_t buffer_get_int16(uint8_t* buff)
{
    return ((uint16_t)buff[0]<<8) | ((uint16_t)buff[1]<<0);
}

/**
 *******************************************************************************
 * \fn          uint16_t buffer_get_uint16(uint8_t* buff)
 * \brief       ��ָ�����������һ��uint16_t����,���������ݰ����ģʽ.
 * \note        .
 * \param[in]   buff     ������ ������2�ֽ���Ч����.
 * \return      uint16_t
 *******************************************************************************
 */
uint16_t buffer_get_uint16(uint8_t* buff)
{
    return ((uint16_t)buff[0]<<8) | ((uint16_t)buff[1]<<0);
}

/**
 *******************************************************************************
 * \fn          float buffer_get_float(uint8_t* buff)
 * \brief       ��ָ�����������һ��float����,���������ݰ����ģʽ.
 * \note        .
 * \param[in]   buff     ������ ������4�ֽ���Ч����.
 * \return      float
 *******************************************************************************
 */
float buffer_get_float(uint8_t* buff)
{
    return ((uint32_t)buff[0]<<24) | ((uint32_t)buff[1]<<16) | ((uint32_t)buff[2]<<8) | ((uint32_t)buff[3]<<0);
}

/**
 *******************************************************************************
 * \fn          void buffer_set_int16(uint8_t* buff,int16_t val)
 * \brief       ��int16_t д��ָ�����������ģʽ.
 * \note        .
 * \param[out]   buff     ������ ������2�ֽ���Ч����.
 * \param[in]    val      ��ת����ֵ.
 * \return       void
 *******************************************************************************
 */
void buffer_set_int16(uint8_t* buff,int16_t val)
{
    buff[0] = (uint8_t)(val>>8);
    buff[1] = (uint8_t)(val>>0);
}

/**
 *******************************************************************************
 * \fn          void buffer_set_uint16(uint8_t* buff,uint16_t val)
 * \brief       ��uint16_t д��ָ�����������ģʽ.
 * \note        .
 * \param[out]   buff     ������ ������2�ֽ���Ч����.
 * \param[in]    val      ��ת����ֵ.
 * \return       void
 *******************************************************************************
 */
void buffer_set_uint16(uint8_t* buff,uint16_t val)
{
    buff[0] = (uint8_t)(val>>8);
    buff[1] = (uint8_t)(val>>0);
}


/**
 *******************************************************************************
 * \fn          void buffer_set_int32(uint8_t* buff,int32_t val)
 * \brief       ��int32_t д��ָ�����������ģʽ.
 * \note        .
 * \param[out]   buff     ������ ������4�ֽ���Ч����.
 * \param[in]    val      ��ת����ֵ.
 * \return       void
 *******************************************************************************
 */
void buffer_set_int32(uint8_t* buff,int32_t val)
{
    buff[0] = (uint8_t)(val>>24);
    buff[1] = (uint8_t)(val>>16);
    buff[2] = (uint8_t)(val>>8);
    buff[3] = (uint8_t)(val>>0);
}

/**
 *******************************************************************************
 * \fn          void buffer_set_uint32(uint8_t* buff,int32_t val)
 * \brief       ��uint32_t д��ָ�����������ģʽ.
 * \note        .
 * \param[out]   buff     ������ ������4�ֽ���Ч����.
 * \param[in]    val      ��ת����ֵ.
 * \return       void
 *******************************************************************************
 */
void buffer_set_uint32(uint8_t* buff,uint32_t val)
{
    buff[0] = (uint8_t)(val>>24);
    buff[1] = (uint8_t)(val>>16);
    buff[2] = (uint8_t)(val>>8);
    buff[3] = (uint8_t)(val>>0);
}

/**
 *******************************************************************************
 * \fn          void buffer_set_float(uint8_t* buff,float val)
 * \brief       ��float д��ָ�����������ģʽ.
 * \note        .
 * \param[out]   buff     ������ ������4�ֽ���Ч����.
 * \param[in]    val      ��ת����ֵ.
 * \return       void
 *******************************************************************************
 */
void buffer_set_float(uint8_t* buff,float val)
{
    buff[0] = (uint8_t)((uint32_t)val>>24);
    buff[1] = (uint8_t)((uint32_t)val>>16);
    buff[2] = (uint8_t)((uint32_t)val>>8);
    buff[3] = (uint8_t)((uint32_t)val>>0);
}

/**
 *******************************************************************************
 * \fn          void buffer_swap(uint8_t* buff,uint8_t len)
 * \brief       buffָ�����ȵ����ݽ�����С��.
 * \note        .
 * \param[out]   buff     ������ .
 * \param[in]    val      ��ת���ĳ���.
 * \return       void
 *******************************************************************************
 */
void buffer_swap(uint8_t* buff,uint8_t len)
{
    uint8_t temp;
    uint8_t i;
    for(i=0;i<len/2;i++)
    {
        temp  = buff[len-1-i];
        buff[len-1-i] = buff[i];
        buff[i] = temp;
    }
}



/**
 *******************************************************************************
 * \fn          uint8_t buffer_checksum(uint8_t* buff, uint32_t len)
 * \brief       �����ۼӺ�.
 * \note        ����8λ�ۼӺ�.
 * \param[in]   buff     ָ������.
 * \param[in]   len      ���ݳ���.
 * \return      uint8_t   У��ֵ
 *******************************************************************************
 */
uint8_t buffer_checksum(uint8_t* buff, uint32_t len)
{
    uint32_t i = 0;
    uint8_t sum = 0;
    for(i=0; i<len; i++)
    {
        sum += buff[i];
    }
    return sum;
}
