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
 * \enum DEV_NUM_e
 * �豸���ö��.
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
 * \fn          int32_t pc_protocol_setswitch(uint8_t* buff)
 * \brief       ���ü̵�������.
 * \note        
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t pc_protocol_setswitch(uint8_t* buff);

/**
 *******************************************************************************
 * \fn          int32_t pc_protocol_ackswitch(uint8_t ch)
 * \brief       ���ؼ̵�������.
 * \note        ���յ���ȡ����ָ��󷵻�.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t pc_protocol_ackswitch(uint8_t ch);

/**
 *******************************************************************************
 * \fn          uint32_t get_dev_state(void)
 * \brief       ��ȡ�豸��Ĭ״̬.
 * \note        .
 * \return      uint32_t ״ֵ̬
 *******************************************************************************
 */
uint32_t get_dev_state(void);

void switch_set_par(uint8_t ch,uint32_t onmin,uint32_t onmax,uint32_t offmin,uint32_t offmax);

void switch_get_par(uint8_t ch,uint32_t* onmin,uint32_t* onmax,uint32_t* offmin,uint32_t* offmax);
#ifdef __cplusplus
}
#endif

#endif