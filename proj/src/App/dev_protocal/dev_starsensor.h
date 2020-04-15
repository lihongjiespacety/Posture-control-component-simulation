#ifndef DEV_STARSENSOR_H
#define DEV_STARSENSOR_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#define STARSENSOR_NUM 1 /*���������ĸ���*/
/**
 * \enum STARSENSOR_Data_t
 * �����ӿ�����.
 */
typedef __packed struct
{
    int32_t i;
    int32_t j;
    int32_t k;
    int32_t q;
    int32_t x_speed;
    int32_t y_speed;
    int32_t z_speed;
    uint8_t exposure;  /*�ع�ʱ��*/
    uint8_t state;    /*�ع�ʱ��*/
    int32_t sec;
    int32_t us;
}STARSENSOR_Data_t;


/**
 *******************************************************************************
 * \fn          int32_t dev_starsensordata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       ���������ӿ�����.
 * \note        ���������ӿ�����.
 * \param[in]   buff     �ӿ�����(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \param[in]   subtype  �ӿ������� һ����ͬ�����豸�����豸���.
 * \param[in]   size     �ӿ����ݳ���(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_starsensordata_handle(uint8_t* buff, uint8_t subtype, uint8_t size);


/**
 *******************************************************************************
 * \fn          int32_t dev_starsensordata_get(MAGTM_Data_t* buff,uint8_t subtype))
 * \brief       ��ȡ��������.
 * \note        .
 * \param[in]   buff     �洢��ȡ��������.
 * \param[in]   subtype  0-n ����豸ָ������һ��.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_starsensordata_get(STARSENSOR_Data_t* buff,uint8_t subtype);

/**
 *******************************************************************************
 * \fn          int32_t dev_starsensortel_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       ��������ң������.
 * \note        ��������ң������.
 * \param[in]   buff     ����.
 * \param[in]   subtype  һ����ͬ�����豸�����豸���.
 * \param[in]   size     �������ݳ���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_starsensortel_handle(uint8_t* buff, uint8_t subtype, uint8_t size);


#ifdef __cplusplus
}
#endif

#endif