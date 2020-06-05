#ifndef DEV_WHEEL_H
#define DEV_WHEEL_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#define WHEEL_NUM 4 /*������ֵĸ���*/
/**
 * \enum WHEEL_Data_t
 * �����ӿ�����.
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
 * ��������ö��.
 */
typedef enum  
{
    WHEEL_TYPE_VRW = 0,   /*�¹�����*/
    WHEEL_TYPE_LY,        /*���·���*/
}WHEEL_Type_e;



/**
 *******************************************************************************
 * \fn          int32_t dev_wheeldata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       ��������ӿ�����.
 * \note        ��������ӿ�����.
 * \param[in]   buff     �ӿ�����(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \param[in]   subtype  �ӿ������� һ����ͬ�����豸�����豸���.
 * \param[in]   size     �ӿ����ݳ���(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_wheeldata_handle(uint8_t* buff, uint8_t subtype, uint8_t size);


/**
 *******************************************************************************
 * \fn          int32_t dev_wheeldata_get(WHEEL_Data_t* buff,uint8_t subtype))
 * \brief       ��ȡ��������.
 * \note        .
 * \param[in]   buff     �洢��ȡ��������.
 * \param[in]   subtype  0-n ����豸ָ������һ��.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_wheeldata_get(WHEEL_Data_t* buff,uint8_t subtype);

/**
 *******************************************************************************
 * \fn          int32_t dev_wheelxtel_handle(uint8_t* buff, uint8_t size)
 * \brief       �������ң������.
 * \note        �������ң������.
 * \param[in]   buff     ����.
 * \param[in]   size     �������ݳ���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_wheelxtel_handle(uint8_t* buff, uint8_t size);

/**
 *******************************************************************************
 * \fn          int32_t dev_wheelytel_handle(uint8_t* buff, uint8_t size)
 * \brief       �������ң������.
 * \note        �������ң������.
 * \param[in]   buff     ����.
 * \param[in]   size     �������ݳ���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_wheelytel_handle(uint8_t* buff, uint8_t size);

/**
 *******************************************************************************
 * \fn          int32_t dev_wheelztel_handle(uint8_t* buff, uint8_t size)
 * \brief       �������ң������.
 * \note        �������ң������.
 * \param[in]   buff     ����.
 * \param[in]   size     �������ݳ���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_wheelztel_handle(uint8_t* buff, uint8_t size);

/**
 *******************************************************************************
 * \fn          int32_t dev_wheeltel_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       �������ң������.
 * \note        �������ң������.
 * \param[in]   buff     ����.
 * \param[in]   subtype  һ����ͬ�����豸�����豸���.
 * \param[in]   size     �������ݳ���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_wheeltel_handle(uint8_t* buff, uint8_t subtype, uint8_t size);

/**
 *******************************************************************************
 * \fn          int32_t dev_wheeltel_tlhandle(uint8_t* buff, uint8_t canid, uint8_t size)
 * \brief       �������ң���.
 * \note        �������ң���.
 * \param[in]   buff     ����.
 * \param[in]   canid  id.
 * \param[in]   size     �������ݳ���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_wheeltel_tlhandle(uint8_t* buff, uint8_t canid, uint8_t size);

void dev_set_wheeltype(uint8_t type);

#ifdef __cplusplus
}
#endif

#endif