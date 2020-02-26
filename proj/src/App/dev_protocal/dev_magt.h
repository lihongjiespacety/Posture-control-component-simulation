#ifndef DEV_MAGT_H
#define DEV_MAGT_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#define MAGT_NUM 6 /*定义磁力矩器个数*/
   
/**
 * \enum MAGT_Data_t
 * 磁力矩器子块内容.
 */
typedef __packed struct
{
    int32_t mx;  /*万分之x*/
    int32_t my;
    int32_t mz;
}MAGT_Data_t;


/**
 *******************************************************************************
 * \fn          int32_t dev_magt_init(void)
 * \brief       初始化磁力矩器输出采集.
 * \note        处理磁强计子块数据.
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_magt_init(void);

/**
 *******************************************************************************
 * \fn          int32_t dev_magtdata_get(MAGT_Data_t* buff,uint8_t subtype,uint8_t swap))
 * \brief       获取磁力矩器数据.
 * \note        .
 * \param[in]   buff     存储获取到的数据.
 * \param[in]   subtype  0-n 多个设备指定是哪一个.
 * \param[in]   swap     0小端 1大端.
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_magtdata_get(MAGT_Data_t* buff,uint8_t subtype,uint8_t swap);


#ifdef __cplusplus
}
#endif

#endif