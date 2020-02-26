/**
 *****************************************************************************        
 * \brief       驱动层(DRIVER)SPI模块(SPI)相关数据结构和接口描述.
 * \details     Copyright (c) 2020,xxx.
 *              All rights reserved.    
 * \file        driver_spi.h 
 * \author      李红洁
 * \version     1.0 
 * \date        2020年2月21日
 * \note        使用前请参考注释.\n
 * \since       李红洁      2020-2-21 16:21:46    1.0     新建 
 * \par 修订记录
 * - 2020-2-21 初始版本
 * \par 资源说明
 * - RAM:              
 * - ROM:
 *****************************************************************************
 */

#ifndef DRIVER_SPI_H
#define DRIVER_SPI_H

#ifdef __cplusplus
 extern "C" {
#endif
 
/** \addtogroup DRIVER 驱动层(DRIVER)
 *  \{
 */
         
/** \addtogroup DRIVER_SPI 驱动层(DRIVER)SPI模块(SPI)
 *  \{
 */
/*****************************************************************************    
 *                                                                           *
 *                             数据结构描述                                  *
 *                                                                          *
 ****************************************************************************/
   
/** \defgroup DRIVER_SPI_data 驱动层(DRIVER)SPI模块(SPI)数据结构 
  * \{
  */

#define SPI_TIMEOUT 10000
    
typedef int32_t (*spi_callbacl_fp)(uint8_t* param) ; /**< 中断服务回调函数        */   


/*******************************************************************************
* \fn          void driver_spi_init(uint8_t master,uint8_t mode,uint32_t speed,spi_callbacl_fp fun)
* \brief       初始化SPI.
* \param[in]   master 0从机 1主机
* \param[in]   mode 0-3
* \param[in]   speed 
* \param[in]   fun  从模式接收回调函数 
* \note        . 
********************************************************************************
*/
void driver_spi_init(uint8_t master,uint8_t mode,uint32_t speed,spi_callbacl_fp fun);

/*******************************************************************************
* \fn          void driver_spi_trans(uint8_t* txbuff,uint8_t* rxbuff, uint8_t len)
* \brief       发送缓冲区同时读缓冲区.
* \param[in]   txbuff 发送缓冲区
* \param[in]   rxbuff 接收缓冲区
* \param[in]   len 发送接收长度 
* \note        . 
********************************************************************************
*/
void driver_spi_trans(uint8_t* txbuff,uint8_t* rxbuff, uint8_t len);



/**
  * \}
  */

/** \defgroup DRIVER_IIC_data 驱动层(DRIVER)IIC模块(IIC)数据结构 
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


