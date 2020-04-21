#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f10x.h"
#include "driver.h"
#include "hal.h"
#include "shell.h"
#include "retarget.h"
#include "osapi_freertos.h"
#include "misc.h"
#include "csp_if_can.h"
#include "can.h"
#include "pc_protocal.h"
#include "cantxrxtask.h"
#include "driver_iic.h"
#include "I2CRoutines.h"
#include "dev_magt.h"
#include "driver_spi.h"
#include "dev_ssoc.h"


#define SPI_TEST 0


#define UART_TASK_STACK_SIZE (256 )
#define UART_TASK_PRIORITY	  ( tskIDLE_PRIORITY + 4 )

#define SHELL_TASK_STACK_SIZE     ( 512 )
#define SHELL_TASK_PRIORITY	  ( tskIDLE_PRIORITY + 3 )

#define CAN_RXTASK_STACK_SIZE (256 )
#define CAN_RXTASK_PRIORITY	  ( tskIDLE_PRIORITY + 6 )

#define CAN_TXTASK_STACK_SIZE (256 )
#define CAN_TXTASK_PRIORITY	  ( tskIDLE_PRIORITY + 6 )

/* Buffer of data to be received by I2C1 */
uint8_t Buffer_Rx1[8];
/* Buffer of data to be transmitted by I2C1 */
uint8_t Buffer_Tx1[8] = {0x5, 0x6,0x8,0xA};
/* Buffer of data to be received by I2C2 */
uint8_t Buffer_Rx2[8];
/* Buffer of data to be transmitted by I2C2 */
uint8_t Buffer_Tx2[8] = {0xF, 0xB, 0xC,0xD};
extern __IO uint8_t Tx_Idx1 , Rx_Idx1;
extern __IO uint8_t Tx_Idx2 , Rx_Idx2;

extern unsigned  prvRxTask(void* params);
extern int kiss_set(int argc); 

uint8_t buff_sum(void* buff,uint8_t len)
{
  uint8_t sum=0;
  uint8_t i;
  uint8_t* p;
  p = buff;
  for(i=0;i<len;i++)
  {
      sum += p[i];
  }
  return sum;
}


uint8_t spi_rdbuff[24]={0};
uint8_t spi_wrbuff1[7]={0x1A,0xCF,0xFC,0x1D,0x04,0x01,0x05};  /*读太敏数据*/
uint8_t spi_wrbuff2[7]={0x1A,0xCF,0xFC,0x1D,0x03,0x01,0x04};  /*滤波后电压*/
uint8_t spi_wrbuff3[7]={0x1A,0xCF,0xFC,0x1D,0x01,0x01,0x02};  /*未滤波电压*/



static void shelltask( void *pvParameters )
{
  MAGT_Data_t magtdata;
  static uint32_t readtime=0;
  HAL_UART_CFG_t cfg;
  cfg.id = HAL_UART_2;
  cfg.baud = HAL_UART_BAUD_115200;
  cfg.datalen = HAL_UART_DATA_8;
  cfg.parity = HAL_UART_PARITY_NONE;
  cfg.stopb = HAL_UART_STOPB_1;
  driver_uart_set(&cfg);
  driver_uart_flush(HAL_UART_2);
  HAL_UART_EnableTx(HAL_UART_2,HAL_UART_INTERRUPT,HAL_UART_DISABLE);
  HAL_UART_EnableTx(HAL_UART_2,HAL_UART_CHECK,HAL_UART_ENABLE);
  setstdiouart(HAL_UART_2);
  printf("任务2启动\r\n");
  while(1)
  {
    shell_exec_shellcmd();
    OsTimeDelay(5);
    
    if((driver_time_getruntime() - readtime) > 1)
    {
        readtime = driver_time_getruntime();
        dev_magtdata_get(&magtdata,0,0);
        printf("duty-mx:%d\r\n",magtdata.mx);
        printf("duty-my:%d\r\n",magtdata.my);
        printf("duty-mz:%d\r\n",magtdata.mz);
    }
#if SPI_TEST
    driver_spi_trans(spi_wrbuff1,spi_rdbuff,sizeof(spi_wrbuff1));
    OsTimeDelay(10);
    memset(spi_rdbuff,0,sizeof(spi_rdbuff));
    driver_spi_trans(spi_rdbuff,spi_rdbuff,17);  /*读数据*/
    OsTimeDelay(10);
    
    driver_spi_trans(spi_wrbuff2,spi_rdbuff,sizeof(spi_wrbuff2));
    OsTimeDelay(10);
    memset(spi_rdbuff,0,sizeof(spi_rdbuff));
    driver_spi_trans(spi_rdbuff,spi_rdbuff,24);  /*读电压*/
    OsTimeDelay(10);
    
    driver_spi_trans(spi_wrbuff3,spi_rdbuff,sizeof(spi_wrbuff3));
    OsTimeDelay(10);
    memset(spi_rdbuff,0,sizeof(spi_rdbuff));
    driver_spi_trans(spi_rdbuff,spi_rdbuff,24);  /*读电压*/
    OsTimeDelay(200);
#endif
    
  }
}


extern void * can_rx_handle(void * parameters);


void csp_debug_hook(csp_color_t color,csp_debug_level_t level, const char *format, va_list args)
{
     OsPrintf_va(color,OSAPI_DEBUG_ERR,format,args);
}


int process_eps_can_request(csp_packet_t* in)
{
    int erro;
    //接收到自定义包通过串口转发
    driver_uart_send(HAL_UART_4, in->data, in->length,2000,&erro);
    OsPrintf(OSAPI_DEBUG_INFO,"can-tianyi:get frame len=%d\r\n", in->length);
    OsPrintf(OSAPI_DEBUG_INFO,"UART4:put frame len=%d\r\n", in->length);
    return 0;

}

/**
  * @brief  Configures NVIC and Vector Table base location.
  * @param  None
  * @retval : None
  */
void NVIC_Configuration(void)
{

    /* 1 bit for pre-emption priority, 3 bits for subpriority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    NVIC_SetPriority(I2C1_EV_IRQn, 0x00); 
    NVIC_EnableIRQ(I2C1_EV_IRQn);

    NVIC_SetPriority(I2C1_ER_IRQn, 0x01); 
    NVIC_EnableIRQ(I2C1_ER_IRQn);
    
    
    NVIC_SetPriority(I2C2_EV_IRQn, 0x00);
    NVIC_EnableIRQ(I2C2_EV_IRQn);

    NVIC_SetPriority(I2C2_ER_IRQn, 0x01); 
    NVIC_EnableIRQ(I2C2_ER_IRQn);
 
}



int main(void)
{ 
  GPIO_InitTypeDef giocfg;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_DeInit(GPIOB);
  
  giocfg.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
  giocfg.GPIO_Speed = GPIO_Speed_10MHz;
  giocfg.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &giocfg);
  
  giocfg.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_15;
  giocfg.GPIO_Speed = GPIO_Speed_10MHz;
  giocfg.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &giocfg);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  SystemCoreClockUpdate();
  
  csp_debug_hook_set(csp_debug_hook);
  driver_time_init();
  driver_can_init(500000);
  driver_can_filter(0, 0, (uint32_t)0 << 21, (uint32_t)0xFF << 21);  /*接收源地址为0的 OBC发过来的数据*/
  driver_can_filter(1, 0, (uint32_t)WHEEL_X_CANID << 21, (uint32_t)0xFF << 21);  /*接收源地址为0的 飞轮发过来的数据*/
  driver_can_filter(2, 0, (uint32_t)WHEEL_Y_CANID << 21, (uint32_t)0xFF << 21);  /*接收源地址为0的 飞轮发过来的数据*/
  driver_can_filter(3, 0, (uint32_t)WHEEL_Z_CANID << 21, (uint32_t)0xFF << 21);  /*接收源地址为0的 飞轮发过来的数据*/
  driver_can_filter(4, 0, (uint32_t)WHEEL_A_CANID << 21, (uint32_t)0xFF << 21);  /*接收源地址为0的 飞轮发过来的数据*/
  //dev_magt_init();
  //driver_iic_init(1,1,0,0,0);
  //NVIC_Configuration();
  //I2C_LowLevel_Init(I2C2);
  //I2C_LowLevel_Init(I2C1);
  //I2C_Slave_BufferReadWrite(I2C1, Interrupt);
#if SPI_TEST
  driver_spi_init(1,3,200000,0);
#else
  driver_spi_init(0,3,200000,dev_ssoc_callback);
#endif
  dev_ssoc_init();
  csp_buffer_init(10, PBUF_MTU);
  xTaskCreate( shelltask, "shell", SHELL_TASK_STACK_SIZE, NULL, SHELL_TASK_PRIORITY, NULL );
  xTaskCreate( uarttask, "uart", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL );
  xTaskCreate( can_rxtask, "canrx", CAN_RXTASK_STACK_SIZE, NULL, CAN_RXTASK_PRIORITY, NULL );
  xTaskCreate( can_txtask, "cantx", CAN_TXTASK_STACK_SIZE, NULL, CAN_TXTASK_PRIORITY, NULL );
  vTaskStartScheduler();
  while(1);
}

void assert_failed(uint8_t* file, uint32_t line)
{
  printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  while(1);
}


void hard_fault_handler_c(unsigned int * hardfault_args) 
{  
    static unsigned int stacked_r0;
    static unsigned int stacked_r1;  
    static unsigned int stacked_r2;  
    static unsigned int stacked_r3;  
    static unsigned int stacked_r12;  
    static unsigned int stacked_lr;  
    static unsigned int stacked_pc;  
    static unsigned int stacked_psr;  
    static unsigned int SHCSR;  
    static unsigned char MFSR;  
    static unsigned char BFSR;   
    static unsigned short int UFSR;  
    static unsigned int HFSR;  
    static unsigned int DFSR;  
    static unsigned int MMAR;  
    static unsigned int BFAR;   
    stacked_r0 = ((unsigned long) hardfault_args[0]);  
    stacked_r1 = ((unsigned long) hardfault_args[1]);  
    stacked_r2 = ((unsigned long) hardfault_args[2]);  
    stacked_r3 = ((unsigned long) hardfault_args[3]);  
    stacked_r12 = ((unsigned long) hardfault_args[4]);  
    /*异常中断发生时，这个异常模式特定的物理R14,即lr被设置成该异常模式将要返回的地址*/  
    stacked_lr = ((unsigned long) hardfault_args[5]);   
    stacked_pc = ((unsigned long) hardfault_args[6]);  
    stacked_psr = ((unsigned long) hardfault_args[7]); 

    MFSR = (*((volatile unsigned char *)(0xE000ED28))); //存储器管理fault状态寄存器   
    BFSR = (*((volatile unsigned char *)(0xE000ED29))); //总线fault状态寄存器   
    UFSR = (*((volatile unsigned short int *)(0xE000ED2A)));//用法fault状态寄存器    
    HFSR = (*((volatile unsigned long *)(0xE000ED2C)));  //硬fault状态寄存器     
    DFSR = (*((volatile unsigned long *)(0xE000ED30))); //调试fault状态寄存器  
    MMAR = (*((volatile unsigned long *)(0xE000ED34))); //存储管理地址寄存器  
    BFAR = (*((volatile unsigned long *)(0xE000ED38))); //总线fault地址寄存器  
    while (1);  
}

void vApplicationMallocFailedHook(void)
{


}