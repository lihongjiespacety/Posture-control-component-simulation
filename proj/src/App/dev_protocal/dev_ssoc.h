#ifndef DEV_SSOC_H
#define DEV_SSOC_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#define SSOC_NUM 1 /*����̫���ĸ���*/
/**
 * \enum SSOC_Data_t
 * ̫���ӿ�����.
 */
typedef __packed struct
{
    float  erfa;
    float  beta;
    uint8_t sta;
    float  cell1;
    float  cell2;
    float  cell3;
    float  cell4;
}SSOC_Data_t;


/**
 *******************************************************************************
 * \fn          int32_t dev_ssocdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       ����̫���ӿ�����.
 * \note        ����̫���ӿ�����.
 * \param[in]   buff     �ӿ�����(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \param[in]   subtype  �ӿ������� һ����ͬ�����豸�����豸���.
 * \param[in]   size     �ӿ����ݳ���(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_ssocdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size);


/**
 *******************************************************************************
 * \fn          int32_t dev_ssocdata_get(SSOC_Data_t* buff,uint8_t subtype))
 * \brief       ��ȡ̫������.
 * \note        .
 * \param[in]   buff     �洢��ȡ��������.
 * \param[in]   subtype  0-n ����豸ָ������һ��.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_ssocdata_get(SSOC_Data_t* buff,uint8_t subtype);

/**
 *******************************************************************************
 * \fn          int32_t dev_ssoctel_handle(uint8_t* buff, uint8_t size)
 * \brief       ����̫��ң������.
 * \note        ����̫��ң������.
 * \param[in]   buff     ����.
 * \param[in]   size     �������ݳ���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_ssoctel_handle(uint8_t* buff, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif