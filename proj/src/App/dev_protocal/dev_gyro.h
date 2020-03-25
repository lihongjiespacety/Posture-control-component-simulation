#ifndef DEV_GYRO_H
#define DEV_GYRO_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#define GYRO_NUM 2 /*�������ݵĸ���*/
/**
 * \enum GYRO_Data_t
 * �����ӿ�����.
 */
typedef __packed struct
{
    int32_t   x_speed;
    int32_t   y_speed;
    int32_t   z_speed;
    int16_t  x_temp;
    int16_t  y_temp;
    int16_t  z_temp;
}GYRO_Data_t;


/**
 *******************************************************************************
 * \fn          int32_t dev_gyrodata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       ���������ӿ�����.
 * \note        ���������ӿ�����.
 * \param[in]   buff     �ӿ�����(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \param[in]   subtype  �ӿ������� һ����ͬ�����豸�����豸���.
 * \param[in]   size     �ӿ����ݳ���(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_gyrodata_handle(uint8_t* buff, uint8_t subtype, uint8_t size);


/**
 *******************************************************************************
 * \fn          int32_t dev_gyrodata_get(GYRO_Data_t* buff,uint8_t subtype))
 * \brief       ��ȡ��������.
 * \note        .
 * \param[in]   buff     �洢��ȡ��������.
 * \param[in]   subtype  0-n ����豸ָ������һ��.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_gyrodata_get(GYRO_Data_t* buff,uint8_t subtype);

/**
 *******************************************************************************
 * \fn          int32_t dev_gyrotel_handle(uint8_t* buff, uint8_t size)
 * \brief       ��������ң������.
 * \note        ��������ң������.
 * \param[in]   buff     ����.
 * \param[in]   size     �������ݳ���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_gyrotel_handle(uint8_t* buff, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif