/**
 *****************************************************************************        
 * \brief       驱动层(DRIVER)IIC模块(IIC)相关数据结构和接口描述.
 * \details     Copyright (c) 2020,xxx.
 *              All rights reserved.    
 * \file        driver_iic.h 
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

#ifndef DRIVER_IIC_H
#define DRIVER_IIC_H

#ifdef __cplusplus
 extern "C" {
#endif
 
/** \addtogroup DRIVER 驱动层(DRIVER)
 *  \{
 */
         
/** \addtogroup DRIVER_IIC 驱动层(DRIVER)IIC模块(IIC)
 *  \{
 */
/*****************************************************************************    
 *                                                                           *
 *                             数据结构描述                                  *
 *                                                                          *
 ****************************************************************************/
   
/** \defgroup DRIVER_IIC_data 驱动层(DRIVER)IIC模块(IIC)数据结构 
  * \{
  */

#define IIC_TIMEOUT 200

/**
  * \}
  */
/*****************************************************************************    
 *                                                                           
 *                             接口函数描述                                   
 *                                                                            
 ****************************************************************************/
/** \defgroup DRIVER_IIC_data 驱动层(DRIVER)IIC模块(IIC)数据结构 
  * \{
  */


/*******************************************************************************
* \fn          void driver_iic_init(uint8_t id,uint8_t mode,uint8_t add1,uint8_t add2,uint8_t speed)
* \brief       初始化IIC.
* \param[in]   id 1-2
* \param[in]   mode 0从机 1主机
* \param[in]   add1 从机地址1
* \param[in]   add2 从机地址2
* \param[in]   speed 0标准速度100k 1快速模式400k
* \note        . 
********************************************************************************
*/
void driver_iic_init(uint8_t id,uint8_t mode,uint8_t add1,uint8_t add2,uint8_t speed);

/*******************************************************************************
* \fn          void driver_iic_send(uint8_t add,uint8_t reg,uint8_t* buff, uint8_t len)
* \brief       写iic.
* \param[in]   id 1-2
* \param[in]   add 从机 地址
* \param[in]   reg 寄存器地址
* \param[in]   buff 待发送数据
* \param[in]   len 待发送数据长度
* \retval      0 成功
* \retval      其他值 失败
* \note        . 
********************************************************************************
*/
int32_t driver_iic_send(uint8_t id,uint8_t add,uint8_t reg,uint8_t* buff, uint8_t len);

/*******************************************************************************
* \fn          void driver_iic_read(uint8_t add,uint8_t reg,uint8_t* buff, uint8_t len)
* \brief       读iic.
* \param[in]   id 1-2
* \param[in]   add 从机 地址
* \param[in]   reg 寄存器地址
* \param[in]   buff 待发送数据
* \param[in]   len 待发送数据长度
* \retval      0 成功
* \retval      其他值 失败
* \note        . 
********************************************************************************
*/
int32_t driver_iic_read(uint8_t id,uint8_t add,uint8_t reg,uint8_t* buff, uint8_t len);

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


