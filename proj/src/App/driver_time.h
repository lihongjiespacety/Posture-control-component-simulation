/**
 *****************************************************************************        
 * \brief       驱动层(DRIVER)时间模块(TIME)相关数据结构和接口描述.
 * \details     Copyright (c) 2017,长沙威胜信息技术有限公司.
 *              All rights reserved.    
 * \file        driver_time.h 
 * \author      李红洁W18900
 * \version     1.0 
 * \date        2017-08-09
 * \note        使用前请参考注释.\n
 *              调用HAL实现,依赖OSAPI,driver_iicrtc.\n
 * \since       李红洁      2017-8-9 18:13:43     1.0     新建 
 * \par 修订记录
 * - 2017-08-09 初始版本
 * \par 资源说明
 * - RAM:              
 * - ROM:
 *****************************************************************************
 */

#ifndef __DRIVER_TIME_H
#define __DRIVER_TIME_H

#ifdef __cplusplus
 extern "C" {
#endif
     
/** \addtogroup DRIVER 驱动层(DRIVER)
 *  \{
 */
         
/** \addtogroup DRIVER_TIME 驱动层(DRIVER)时间模块(TIME)
 *  \{
 */
/*****************************************************************************    
 *                                                                           *
 *                             数据结构描述                                  *
 *                                                                          *
 ****************************************************************************/
/** \defgroup DRIVER_TIME_data 驱动层(DRIVER)时间模块(TIME)数据结构 
  * \{
  */
     
#define DRIVER_TIME_SYNCPERIOD  (60*10)     /**< 时间同步检查间隔时间  */
#define DRIVER_TIME_SEMTIMEOUT  (1000*4)    /**< 时间处理信号量超时时间 */
     
/**
  * \}
  */
/*****************************************************************************    
 *                                                                           
 *                             接口函数描述                                   
 *                                                                            
 ****************************************************************************/

/** \defgroup DRIVER_TIME_if 驱动层(DRIVER)时间模块(TIME)接口
  * \{
  */
     
/**
 *****************************************************************************
 * \fn          int driver_time_init(void)
 * \brief       初始化时间驱动.
 * \retval      0 操作成功
 * \retval      其他值 操作失败
 * \note        .
 *****************************************************************************
 */ 
int driver_time_init(void);

/**
 *****************************************************************************
 * \fn          void driver_time_sechandle(void)
 * \brief       软件时间更新回调函数,1秒钟调用一次.
 * \note        .
 *****************************************************************************
 */ 
void driver_time_sechandle(void);

/**
 *****************************************************************************
 * \fn          int driver_time_settime(time_t t)
 * \brief       设置时间.
 * \param[in]   t  要设置的时间戳.
 * \retval      0 操作成功
 * \retval      其他值 操作失败
 * \note        .
 *****************************************************************************
 */ 
int driver_time_settime(time_t t);

/**
 *****************************************************************************
 * \fn          int driver_time_gettm(struct tm *p)
 * \brief       获取结构体时间.
 * \param[out]  p  存储获取的时间结构体.
 * \retval      0 操作成功
 * \retval      其他值 操作失败
 * \note        .
 *****************************************************************************
 */ 
int driver_time_gettm(struct tm *p);

/**
 *****************************************************************************
 * \fn          int driver_time_gettm(struct tm *p)
 * \brief       获取结构体时间.
 * \param[out]  p  存储获取的时间结构体.
 * \retval      0 操作成功
 * \retval      其他值 操作失败
 * \note        .
 *****************************************************************************
 */ 
int driver_time_settm(struct tm *p);
/**
 *****************************************************************************
 * \fn          time_t driver_time_gettime(void)
 * \brief       获取时间戳时间.
 * \return      time_t 返回软件时间戳
 * \note        .
 *****************************************************************************
 */ 
time_t driver_time_gettime(void);

/**
 *****************************************************************************
 * \fn          time_t driver_time_getruntime(void)
 * \brief       获取系统运行时间.
 * \return      time_t 返回系统运行时间
 * \note        .
 *****************************************************************************
 */ 
time_t driver_time_getruntime(void);

/**
 *****************************************************************************
 * \fn          int driver_time_getrtctm(struct tm *p)
 * \brief       获取硬件RTC结构体时间.
 * \retval      0 操作成功
 * \retval      其他值 操作失败
 * \note        .
 *****************************************************************************
 */ 
int driver_time_getrtctm(struct tm *p);
/**
 *****************************************************************************
 * \fn          struct tm * driver_time_localtime_r(const time_t *pt, struct tm *ptm)
 * \brief       将time_t时间戳转换成结构体时间struct tm,线程安全版本.
 * \param[in]   pt  指向需要转换的时间戳.
 * \param[out]  ptm 保存转换结果,调用者提供存储空间.
 * \return      struct tm* 返回指向转换结果的指针,如果返回0表示操作失败.
 * \note        .
 *****************************************************************************
 */ 
struct tm * driver_time_localtime_r(const time_t *pt, struct tm *ptm);

/**
 *****************************************************************************
 * \fn          time_t driver_time_mktime_r(struct tm * t)
 * \brief       将结构体时间struct tm转换成time_t时间戳,线程安全版本.
 * \param[in]   t  指向需要转换的时间结构体.
 * \return      time_t 返回转换后的时间戳,返回0表示操作失败. 
 * \note        .
 *****************************************************************************
 */ 
time_t driver_time_mktime_r(struct tm * t);

/**
  * \}
  */

/** \defgroup DRIVER_TIME_samp 驱动层(DRIVER)时间模块(TIME)使用样例
  * \{
  */

/*****************************************************************************    
 *                                                                           
 *                             使用样例                                   
 *                                                                            
 ****************************************************************************/

/**
 *****************************************************************************
 * \fn          void driver_time_samp(void)
 * \brief       使用样例.
 * \par   参见  【示例】标签下的【driver_time_samp.c】
 *****************************************************************************
 */
void driver_time_samp(void);

/**
 *\example driver_time_samp.c
 * 驱动层(DRIVER)时间模块(TIME)使用样例.\n
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




