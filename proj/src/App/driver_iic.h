/**
 *****************************************************************************        
 * \brief       ������(DRIVER)IICģ��(IIC)������ݽṹ�ͽӿ�����.
 * \details     Copyright (c) 2020,xxx.
 *              All rights reserved.    
 * \file        driver_iic.h 
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

#ifndef DRIVER_IIC_H
#define DRIVER_IIC_H

#ifdef __cplusplus
 extern "C" {
#endif
 
/** \addtogroup DRIVER ������(DRIVER)
 *  \{
 */
         
/** \addtogroup DRIVER_IIC ������(DRIVER)IICģ��(IIC)
 *  \{
 */
/*****************************************************************************    
 *                                                                           *
 *                             ���ݽṹ����                                  *
 *                                                                          *
 ****************************************************************************/
   
/** \defgroup DRIVER_IIC_data ������(DRIVER)IICģ��(IIC)���ݽṹ 
  * \{
  */

#define IIC_TIMEOUT 200

/**
  * \}
  */
/*****************************************************************************    
 *                                                                           
 *                             �ӿں�������                                   
 *                                                                            
 ****************************************************************************/
/** \defgroup DRIVER_IIC_data ������(DRIVER)IICģ��(IIC)���ݽṹ 
  * \{
  */


/*******************************************************************************
* \fn          void driver_iic_init(uint8_t id,uint8_t mode,uint8_t add1,uint8_t add2,uint8_t speed)
* \brief       ��ʼ��IIC.
* \param[in]   id 1-2
* \param[in]   mode 0�ӻ� 1����
* \param[in]   add1 �ӻ���ַ1
* \param[in]   add2 �ӻ���ַ2
* \param[in]   speed 0��׼�ٶ�100k 1����ģʽ400k
* \note        . 
********************************************************************************
*/
void driver_iic_init(uint8_t id,uint8_t mode,uint8_t add1,uint8_t add2,uint8_t speed);

/*******************************************************************************
* \fn          void driver_iic_send(uint8_t add,uint8_t reg,uint8_t* buff, uint8_t len)
* \brief       дiic.
* \param[in]   id 1-2
* \param[in]   add �ӻ� ��ַ
* \param[in]   reg �Ĵ�����ַ
* \param[in]   buff ����������
* \param[in]   len ���������ݳ���
* \retval      0 �ɹ�
* \retval      ����ֵ ʧ��
* \note        . 
********************************************************************************
*/
int32_t driver_iic_send(uint8_t id,uint8_t add,uint8_t reg,uint8_t* buff, uint8_t len);

/*******************************************************************************
* \fn          void driver_iic_read(uint8_t add,uint8_t reg,uint8_t* buff, uint8_t len)
* \brief       ��iic.
* \param[in]   id 1-2
* \param[in]   add �ӻ� ��ַ
* \param[in]   reg �Ĵ�����ַ
* \param[in]   buff ����������
* \param[in]   len ���������ݳ���
* \retval      0 �ɹ�
* \retval      ����ֵ ʧ��
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


