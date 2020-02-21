#include "can.h"
#include "driver_can.h"


/**
 *******************************************************************************
 * \fn          void can_txtask(void* pvParameters)
 * \brief       CAN��������.
 * \note        �ȴ����� ����CAN��Ϣ.
 *******************************************************************************
 */
void can_txtask(void* pvParameters)
{
    driver_can_status_t status_t;
    while(1)
    {
        /*������*/
        driver_can_getstatus(&status_t);
        if(((status_t.esr & 0x06) != 0) || (status_t.rcv_err >= 127) || (status_t.send_err >= 127))
        {
            /*���󱻶���־����ʱ���³�ʼ��*/
            driver_can_init((uint32_t)500000);
        }
        
        driver_can_sendloop();
    }
}

/**
 *******************************************************************************
 * \fn          void can_rxtask(void* pvParameters)
 * \brief       CAN��������.
 * \note        �ȴ�����.
 *******************************************************************************
 */
void can_rxtask(void* pvParameters)
{
    while(1)
    {
      
        can_rx_handle(0);   /*�ȴ����� ���治��Ҫ��osdelay*/
    }
}
