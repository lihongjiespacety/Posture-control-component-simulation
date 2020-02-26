#ifndef DEV_SSOC_H
#define DEV_SSOC_H

#ifdef __cplusplus
 extern "C" {
#endif

#define CMD_LEN 7
#define DATA_LEN 17
#define VOL_LEN 24

#define SSOC_NUM 1 /*定义太敏的个数*/
   
   
/**
 * \enum SSOC_Data_t
 * 太敏子块内容.
 */
typedef __packed struct
{
    float  erfa;
    float  beta;
    uint8_t sta;
    float  cell1;
    float  cell2;
    float  cell3;
    float  cell4;
}SSOC_Data_t;

/**
 * \enum SSOC_State_e
 * 太敏数据处理状态枚举.
 */
typedef enum
{
    SSOC_RCVCMD = 0,    /*正在接收命令*/
    SSOC_SENDDATA = 1,  /*正在发送数据响应*/
}SSOC_State_e;


/**
 * \enum SSOC_Ctrl_t
 * 太敏数据处理状态枚举.
 */
typedef __packed struct
{
    SSOC_State_e state;      /*状态*/
    uint8_t rxindex;         /*接收索引*/
    uint8_t rxbuff[CMD_LEN]; /*接收缓冲区*/
    uint8_t txindex;         /*发送索引*/
    uint8_t txlen;           /*发送数据长度*/
    uint8_t* txbuff;         /*发送缓冲区指针*/
}SSOC_Ctrl_t;

/**
 *******************************************************************************
 * \fn          int32_t dev_ssoc_init(void)
* \brief       太敏设备模块初始化.
 * \note        .
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_ssoc_init(void);

/**
 *******************************************************************************
 * \fn          int32_t dev_ssocdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       处理太敏子块数据.
 * \note        处理太敏子块数据.
 * \param[in]   buff     子块内容(不包括子块长度 子块类型 子块子类型).
 * \param[in]   subtype  子块子类型 一般多个同类型设备代表设备序号.
 * \param[in]   size     子块内容长度(不包括子块长度 子块类型 子块子类型).
 * \retval      0 成功
 * \retval      其他值 失败
 *******************************************************************************
 */
int32_t dev_ssocdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size);

/**
 *******************************************************************************
 * \fn          int32_t dev_ssoc_callback(uint8_t* param);
 * \brief       SPI字节接收回调函数.
 * \note        采用状态机处理.
 * \param[in]   param 指向接收到的数据或者需要回写的数据.
 * \retval      0 不需要回写
 * \retval      1 需要回写
 *******************************************************************************
 */
int32_t dev_ssoc_callback(uint8_t* param);


#ifdef __cplusplus
}
#endif

#endif