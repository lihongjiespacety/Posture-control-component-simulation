#ifndef CANTXTASK_H
#define CANTXTASK_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/**
 *******************************************************************************
 * \fn          void can_txtask(void* pvParameters)
 * \brief       CAN��������.
 * \note        �ȴ����� ����CAN��Ϣ.
 *******************************************************************************
 */
void can_txtask(void* pvParameters);

/**
 *******************************************************************************
 * \fn          void can_rxtask(void* pvParameters)
 * \brief       CAN��������.
 * \note        �ȴ�����.
 *******************************************************************************
 */
void can_rxtask(void* pvParameters);

#ifdef __cplusplus
}
#endif

#endif