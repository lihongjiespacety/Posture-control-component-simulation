/**
 *****************************************************************************        
 * \brief       ������(DRIVER)SPIģ��(SPI)������ݽṹ�ͽӿ�����.
 * \details     Copyright (c) 2020,xxx.
 *              All rights reserved.    
 * \file        driver_spi.h 
 * \author      ����
 * \version     1.0 
 * \date        2020��2��21��
 * \note        ʹ��ǰ��ο�ע��.\n
 * \since       ����      2020-2-21 16:21:46    1.0     �½� 
 * \par �޶���¼
 * - 2020-2-21 ��ʼ�汾
 * \par ��Դ˵��
 * - RAM:              
 * - ROM:
 *****************************************************************************
 */

#ifndef DRIVER_SPI_H
#define DRIVER_SPI_H

#ifdef __cplusplus
 extern "C" {
#endif
 
/** \addtogroup DRIVER ������(DRIVER)
 *  \{
 */
         
/** \addtogroup DRIVER_SPI ������(DRIVER)SPIģ��(SPI)
 *  \{
 */
/*****************************************************************************    
 *                                                                           *
 *                             ���ݽṹ����                                  *
 *                                                                          *
 ****************************************************************************/
   
/** \defgroup DRIVER_SPI_data ������(DRIVER)SPIģ��(SPI)���ݽṹ 
  * \{
  */

#define SPI_TIMEOUT 10000
    
typedef int32_t (*spi_callbacl_fp)(uint8_t* param) ; /**< �жϷ���ص�����        */   


/*******************************************************************************
* \fn          void driver_spi_init(uint8_t master,uint8_t mode,uint32_t speed,spi_callbacl_fp fun)
* \brief       ��ʼ��SPI.
* \param[in]   master 0�ӻ� 1����
* \param[in]   mode 0-3
* \param[in]   speed 
* \param[in]   fun  ��ģʽ���ջص����� 
* \note        . 
********************************************************************************
*/
void driver_spi_init(uint8_t master,uint8_t mode,uint32_t speed,spi_callbacl_fp fun);

/*******************************************************************************
* \fn          void driver_spi_trans(uint8_t* txbuff,uint8_t* rxbuff, uint8_t len)
* \brief       ���ͻ�����ͬʱ��������.
* \param[in]   txbuff ���ͻ�����
* \param[in]   rxbuff ���ջ�����
* \param[in]   len ���ͽ��ճ��� 
* \note        . 
********************************************************************************
*/
void driver_spi_trans(uint8_t* txbuff,uint8_t* rxbuff, uint8_t len);



/**
  * \}
  */

/** \defgroup DRIVER_IIC_data ������(DRIVER)IICģ��(IIC)���ݽṹ 
  * \{
  */



/**
  * \}
  */

/**
  * \}
  */

/**
  * \}
  */

#ifdef __cplusplus
}
#endif

#endif


