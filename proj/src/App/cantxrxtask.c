#include "can.h"
#include "driver_can.h"


/**
 *******************************************************************************
 * \fn          void can_txtask(void* pvParameters)
 * \brief       CAN发送任务.
 * \note        等待队列 发送CAN消息.
 *******************************************************************************
 */
void can_txtask(void* pvParameters)
{
    driver_can_status_t status_t;
    while(1)
    {
        /*错误处理*/
        driver_can_getstatus(&status_t);
        if(((status_t.esr & 0x06) != 0) || (status_t.rcv_err >= 127) || (status_t.send_err >= 127))
        {
            /*错误被动标志以上时重新初始化*/
            driver_can_init((uint32_t)500000);
        }
        
        driver_can_sendloop();
    }
}

/**
 *******************************************************************************
 * \fn          void can_rxtask(void* pvParameters)
 * \brief       CAN接收任务.
 * \note        等待队列.
 *******************************************************************************
 */
void can_rxtask(void* pvParameters)
{
    while(1)
    {
      
        can_rx_handle(0);   /*等待队列 后面不需要再osdelay*/
    }
}
