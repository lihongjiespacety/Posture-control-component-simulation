/**
 ********************************************************************************        
 * \brief      驱动层(DRIVER)CAN模块(CAN)相关接口描述.
 * \details    Copyright (c) 2019,spacety.
 *             All rights reserved.   
 * \file       driver_can.h 
 * \author     lihongjie@spacety.cn
 * \version    1.0 
 * \date       2019-08-30
 * \note       使用前请参考注释.\n
 * \since      lihongjie@spacety.cn 2019-8-30 1.0  新建
 * \par 修订记录
 * - 2019-08-309 初始版本
 * \par 资源说明
 * - RAM:              
 * - ROM:
 ********************************************************************************
 */

#ifndef DRIVER_CAN_H
#define DRIVER_CAN_H

#ifdef __cplusplus
extern "C" {
#endif
  
  
#include <stdint.h>
  /** \addtogroup DRIVER 驱动层(DRIVER)
  *  \{
  */
  
  /** \addtogroup DRIVER_CAN 驱动层(DRIVER)CAN模块(CAN)
  *  \{
  */
  /*******************************************************************************    
  *                                                                             *
  *                             数据结构描述                                    *
  *                                                                             *
  ******************************************************************************/
  
  /** \defgroup DRIVER_CAN_DATA 驱动层(DRIVER)CAN模块(CAN)数据结构 
  * \{
  */
  

#define DRIVER_CAN_BTR_SJW  (uint32_t)4    /**< 重新同步跳跃时间单元 1-4 值越大对波特率容差越大*/
#define DRIVER_CAN_BTR_TS2  (uint32_t)5    /**< 时间段2：相位缓冲段2 的时间单元 1-8 */

/* DRIVER_CAN_BTR_TS1决定采样点  TSEG2 >= SJW  
采样点 = (1+DRIVER_CAN_BTR_TS2)/(1+DRIVER_CAN_BTR_TS1+DRIVER_CAN_BTR_TS2) = 4/8 = 50%
采样点为50%时拥有最大的晶振容差
*/
#define DRIVER_CAN_BTR_TS1  (uint32_t)7   /**< 时间段1: 相位缓冲段1 + PROP_SEG传播时间段(PTS)的时间单元 1-16 */



#define CAN_RXQUEUE_LEN (PBUF_MTU/4)  /**< CAN接收缓冲区大小 */
#define CAN_TXQUEUE_LEN (PBUF_MTU/4)  /**< CAN发送缓冲区大小 */

#define CAN1_RX1_PRI  14   /**< CAN1 FIFO1接收中断优先级 */
#define CAN1_SCE_PRI  14   /**< CAN1 错误中断优先级 */
#define CAN1_RX0_PRI  14   /**< CAN1 FIFO0接收中断优先级*/
#define CAN1_TX_PRI   14   /**< CAN1 错误中断优先级*/


  
#define DRIVER_CAN_DELAY_CHECK (uint32_t)1  /**< CAN数据查询间隔时间 */
#define DRIVER_CAN_RXBUFFSIZE (uint16_t)20  /**< CAN接收缓冲区大小   */
#define DRIVER_CAN_TXBUFFSIZE (uint16_t)20  /**< CAN发送缓冲区大小   */
    
#define CAN_DATA_IDE_BIT 1
#define CAN_DATA_RTR_BIT 0
    
#define CAN_TIMEOUT (int32_t)20000    /**< 超时时间 单位uS*/
 /**
 * \struct driver_can_data_t
 * CAN驱动配置结构体.
 */
typedef  struct
{
    uint8_t type_u8;              /**< bit1 = ide  bit0 = rtr      */ 
    uint8_t len_u8;               /**< 有效数据长度                */ 
    uint8_t buff_au8[8];          /**< 数据                        */ 
    uint32_t id_u32;              /**< id */
}driver_can_data_t;
  
 /**
 * \struct driver_can_status_t
 * CAN状态结构体.
 */
typedef __packed struct 
{
    uint8_t send_err;              /**< 发送错误帧计数     */ 
    uint8_t rcv_err;               /**< 接收错误帧计数     */ 
    uint32_t send_frames;          /**< 发送帧数           */ 
    uint32_t rcv_frames;           /**< 接受帧数           */
    uint32_t esr;                  /**< 状态寄存器         */
}driver_can_status_t;


/**
 * \struct driver_can_t
 * CAN驱动配置结构体.
 */
typedef struct 
{
    uint16_t rx_in;              /**< 接收缓冲区写入指针       */ 
    uint16_t rx_out;             /**< 接收缓冲区读出指针       */ 
    uint16_t rx_len;             /**< 接收缓冲区有效数据大小   */ 
    uint16_t tx_in;              /**< 发送缓冲区写入指针       */ 
    uint16_t tx_out;             /**< 发送缓冲区读出指针      */ 
    uint16_t tx_len;             /**< 发送缓冲区有效数据大小   */ 
    uint16_t rxbuf_len;          /**< 接收缓冲区大小           */   
    uint16_t txbuf_len;          /**< 发送缓冲区大小          */ 
    driver_can_data_t *rx_buf;   /**< 接收缓冲区              */ 
    driver_can_data_t *tx_buf;   /**< 发送缓冲区              */ 
}driver_can_t;

  
  /**
  * \}
  */
  /*******************************************************************************    
  *                                                                           
  *                             接口函数描述                                   
  *                                                                            
  ******************************************************************************/
  
  /** \defgroup DRIVER_CAN_if 驱动层(DRIVER)CAN模块(CAN)接口
  * \{
  */
 
/**
********************************************************************************
* \fn          void driver_can_init(uint32_t baudrate)
* \brief       初始化CAN.
* \param[in]   baudrate 波特率.
* \note        . 
********************************************************************************
*/
void driver_can_init(uint32_t baudrate);
    

/**
********************************************************************************
* \fn          void driver_can_filter(uint_t mode,uint32_t id,uint32_t mask)
* \brief       设置CAN过滤.
* \param[in]   ch 通道0-13.
* \param[in]   mode 0:MASK模式 1:list模式.
* \param[in]   id id.
* \param[in]   mask 掩码.
* \note        . 
********************************************************************************
*/
void driver_can_filter(uint8_t ch,uint8_t mode,uint32_t id,uint32_t mask);

 /**
 *****************************************************************************
 * \fn          int8_t driver_can_send(driver_can_data_t data, uint32_t timeout)
 * \brief       发送CAN数据(写入缓冲区).
 * \note        应用调用该函数调写数据到发送缓冲区,缓冲区满等待. 
 * \param[in]   data 指向can_data结构体实例的指针 \ref driver_can_data_t
 * \param[in]   timeout 设定超时时间 0表示一直等待
 * \retval      0 写成功
 * \retval      1 写失败
 *****************************************************************************
 */
int8_t driver_can_send(driver_can_data_t data, uint32_t timeout);

/**
 *****************************************************************************
 * \fn          int8_t driver_can_recv(driver_can_data_t* data, uint32_t timeout)
 * \brief       从接收缓冲区读数据.
 * \note        应用函数调用该函数从接收缓冲区读数据,可以指定超时时间. 
 * \param[out]  data 指向can_data结构体实例的指针 \ref driver_can_data_t
 * \param[in]   timeout 指定超时时间 0表示一直等待 
 * \retval      int8_t 0成功 其他值失败.
 *****************************************************************************
 */
int8_t driver_can_recv(driver_can_data_t* data, uint32_t timeout);

    
/**
 *****************************************************************************
 * \fn          int8_t driver_can_sendloop(void)
 * \brief       如果发送消息邮箱有空,且有发送数据,则将发送缓冲区数据写入发送消息邮箱.
 * \note        应用周期调用该函数进行发送处理. 
 * \retval      0 有发送
 * \retval      1 无发送
 *****************************************************************************
 */
int8_t driver_can_sendloop(void);



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


