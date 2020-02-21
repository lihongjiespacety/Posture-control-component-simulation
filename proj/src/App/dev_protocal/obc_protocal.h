#ifndef OBC_PROTOCAL_H
#define OBC_PROTOCAL_H

#ifdef __cplusplus
 extern "C" {
#endif
   

/**
 *******************************************************************************
 * \fn          int32_t obc_protocol_handle_data(uint8_t canid, uint8_t* buff, uint32_t size)
 * \brief       OBC������������֡����.
 * \note        ��������֡����.
 * \param[in]   canid    ģ���can�豸��id.
 * \param[in]   buff     ָ����������.
 * \param[in]   size     ���ݵĳ���.
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t obc_protocol_handle_data(uint8_t canid, uint8_t* buff, uint32_t size);


#ifdef __cplusplus
}
#endif

#endif