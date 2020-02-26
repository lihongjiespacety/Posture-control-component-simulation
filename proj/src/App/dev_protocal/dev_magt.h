#ifndef DEV_MAGT_H
#define DEV_MAGT_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#define MAGT_NUM 6 /*���������������*/
   
/**
 * \enum MAGT_Data_t
 * ���������ӿ�����.
 */
typedef __packed struct
{
    int32_t mx;  /*���֮x*/
    int32_t my;
    int32_t mz;
}MAGT_Data_t;


/**
 *******************************************************************************
 * \fn          int32_t dev_magt_init(void)
 * \brief       ��ʼ��������������ɼ�.
 * \note        �����ǿ���ӿ�����.
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_magt_init(void);

/**
 *******************************************************************************
 * \fn          int32_t dev_magtdata_get(MAGT_Data_t* buff,uint8_t subtype,uint8_t swap))
 * \brief       ��ȡ������������.
 * \note        .
 * \param[in]   buff     �洢��ȡ��������.
 * \param[in]   subtype  0-n ����豸ָ������һ��.
 * \param[in]   swap     0С�� 1���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_magtdata_get(MAGT_Data_t* buff,uint8_t subtype,uint8_t swap);


#ifdef __cplusplus
}
#endif

#endif