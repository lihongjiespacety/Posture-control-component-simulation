#ifndef DEV_MAGTM_H
#define DEV_MAGTM_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#define MAGTM_NUM 1 /*�����ǿ�Ƶĸ���*/
/**
 * \enum MAGTM_Data_t
 * ��ǿ���ӿ�����.
 */
typedef __packed struct
{
    int32_t mx;
    int32_t my;
    int32_t mz;
}MAGTM_Data_t;

/**
 * \struct zju_magm_ptm_t
 * ����ǿ��ң�����ݽṹ.
 */
typedef struct zju_magm_ptm 
{
	int32_t x_i32;      /**<  nT    */
	int32_t y_i32;      /**<  nT    */
	int32_t z_i32;      /**<  nT    */
} zju_magm_ptm_t;


/**
 *******************************************************************************
 * \fn          int32_t dev_magtmdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       �����ǿ���ӿ�����.
 * \note        �����ǿ���ӿ�����.
 * \param[in]   buff     �ӿ�����(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \param[in]   subtype  �ӿ������� һ����ͬ�����豸�����豸���.
 * \param[in]   size     �ӿ����ݳ���(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_magtmdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size);


/**
 *******************************************************************************
 * \fn          int32_t dev_magtmdata_get(MAGTM_Data_t* buff,uint8_t subtype))
 * \brief       ��ȡ��ǿ������.
 * \note        .
 * \param[in]   buff     �洢��ȡ��������.
 * \param[in]   subtype  0-n ����豸ָ������һ��.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_magtmdata_get(MAGTM_Data_t* buff,uint8_t subtype);

/**
 *******************************************************************************
 * \fn          int32_t dev_magtmtel_handle(uint8_t* buff, uint8_t size)
 * \brief       �����ǿ��ң������.
 * \note        �����ǿ��ң������.
 * \param[in]   buff     ����.
 * \param[in]   size     �������ݳ���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_magtmtel_handle(uint8_t* buff, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif