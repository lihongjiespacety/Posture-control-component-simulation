#ifndef PC_PROTOCAL_H
#define PC_PROTOCAL_H

#ifdef __cplusplus
 extern "C" {
#endif
   
void uarttask(void *pvParameters );

/**
 * \enum DATA_TYPE_e
 * ����֡�ӿ�����.
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
 * ����֡�ӿ�����.
 */
typedef enum
{
    CMD_TEST = 0,
}CMD_TYPE_e;


/**
 *******************************************************************************
 * \fn          int32_t pc_protocol_initbuffer(uint8_t* buff, uint32_t* size,uint32_t bufflen)
 * \brief       ���һ����pcͨѶ��ʼ���Ŀհ�.
 * \note        .
 * \param[in]   buff     ָ��֡����������,���ݿ��ܱ���д �����߷���.
 * \param[in]   size     ��Ч֡���ݳ���,���ݿ��ܻᱻ��д.
 * \param[in]   bufflen  ��Ч֡���ݳ���,���ݿ��ܻᱻ��д ������ָ��.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t pc_protocol_initbuffer(uint8_t* buff, uint32_t* size,uint32_t bufflen);

   
/**
 *******************************************************************************
 * \fn          int32_t pc_protocol_apendbuffer(uint8_t* buff, uint32_t* size,uint32_t bufflen,void* adddata,uint32_t adddatalen,uint8_t type, uint8_t subtype)
 * \brief       ���һ�����ݿ�.
 * \note        .
 * \param[in]   buff     ָ��֡����������,���ݿ��ܱ���д �����߷���.
 * \param[in]   size     ��Ч֡���ݳ���,���ݿ��ܻᱻ��д.
 * \param[in]   adddata  ����ӵ�����.
 * \param[in]   adddatalen  �������������.
 * \param[in]   type  ����.
 * \param[in]   subtype  ������.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t pc_protocol_apendbuffer(uint8_t* buff, uint32_t* size,uint32_t bufflen,void* adddata,uint32_t adddatalen,uint8_t type, uint8_t subtype);


/**
 *******************************************************************************
 * \fn          int32_t pc_protocol_ackhandle(void)
 * \brief       PC��������֡���ݻ�Ӧ����(�������ݰ�).
 * \note        ���յ��������ݺ���øú�������.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t pc_protocol_ackhandle(void);

/**
 *******************************************************************************
 * \fn          uint8_t get_dev_state(void)
 * \brief       ��ȡ�豸��Ĭ״̬.
 * \note        .
 * \return      uint8_t ״ֵ̬
 *******************************************************************************
 */
uint8_t get_dev_state(void);


#ifdef __cplusplus
}
#endif

#endif