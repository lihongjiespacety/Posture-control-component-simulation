#ifndef DEV_SSOC_H
#define DEV_SSOC_H

#ifdef __cplusplus
 extern "C" {
#endif

#define CMD_LEN 7
#define DATA_LEN 17
#define VOL_LEN 24

#define SSOC_NUM 1 /*����̫���ĸ���*/
   
   
/**
 * \enum SSOC_Data_t
 * ̫���ӿ�����.
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
 * ̫�����ݴ���״̬ö��.
 */
typedef enum
{
    SSOC_RCVCMD = 0,    /*���ڽ�������*/
    SSOC_SENDDATA = 1,  /*���ڷ���������Ӧ*/
}SSOC_State_e;


/**
 * \enum SSOC_Ctrl_t
 * ̫�����ݴ���״̬ö��.
 */
typedef __packed struct
{
    SSOC_State_e state;      /*״̬*/
    uint8_t rxindex;         /*��������*/
    uint8_t rxbuff[CMD_LEN]; /*���ջ�����*/
    uint8_t txindex;         /*��������*/
    uint8_t txlen;           /*�������ݳ���*/
    uint8_t* txbuff;         /*���ͻ�����ָ��*/
}SSOC_Ctrl_t;

/**
 *******************************************************************************
 * \fn          int32_t dev_ssoc_init(void)
* \brief       ̫���豸ģ���ʼ��.
 * \note        .
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_ssoc_init(void);

/**
 *******************************************************************************
 * \fn          int32_t dev_ssocdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
 * \brief       ����̫���ӿ�����.
 * \note        ����̫���ӿ�����.
 * \param[in]   buff     �ӿ�����(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \param[in]   subtype  �ӿ������� һ����ͬ�����豸�����豸���.
 * \param[in]   size     �ӿ����ݳ���(�������ӿ鳤�� �ӿ����� �ӿ�������).
 * \retval      0 �ɹ�
 * \retval      ����ֵ ʧ��
 *******************************************************************************
 */
int32_t dev_ssocdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size);

/**
 *******************************************************************************
 * \fn          int32_t dev_ssoc_callback(uint8_t* param);
 * \brief       SPI�ֽڽ��ջص�����.
 * \note        ����״̬������.
 * \param[in]   param ָ����յ������ݻ�����Ҫ��д������.
 * \retval      0 ����Ҫ��д
 * \retval      1 ��Ҫ��д
 *******************************************************************************
 */
int32_t dev_ssoc_callback(uint8_t* param);


#ifdef __cplusplus
}
#endif

#endif