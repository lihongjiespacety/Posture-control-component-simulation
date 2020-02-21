#ifndef CANTXTASK_H
#define CANTXTASK_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/**
 *******************************************************************************
 * \fn          void can_txtask(void* pvParameters)
 * \brief       CAN发送任务.
 * \note        等待队列 发送CAN消息.
 *******************************************************************************
 */
void can_txtask(void* pvParameters);

/**
 *******************************************************************************
 * \fn          void can_rxtask(void* pvParameters)
 * \brief       CAN接收任务.
 * \note        等待队列.
 *******************************************************************************
 */
void can_rxtask(void* pvParameters);

#ifdef __cplusplus
}
#endif

#endif