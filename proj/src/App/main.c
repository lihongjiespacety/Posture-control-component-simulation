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

#define UART_TASK_STACK_SIZE (256 )
#define UART_TASK_PRIORITY	  ( tskIDLE_PRIORITY + 4 )

#define SHELL_TASK_STACK_SIZE     ( 512 )
#define SHELL_TASK_PRIORITY	  ( tskIDLE_PRIORITY + 3 )

#define CAN_RXTASK_STACK_SIZE (256 )
#define CAN_RXTASK_PRIORITY	  ( tskIDLE_PRIORITY + 6 )

#define CAN_TXTASK_STACK_SIZE (256 )
#define CAN_TXTASK_PRIORITY	  ( tskIDLE_PRIORITY + 6 )



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

uint8_t ver_buff[8]={0x11,0x22,0x33,0x44,0x00,0x00,0x00,0x00};
uint8_t tm_buff[40]=
{0x00,0x25,0x33,0x44,0x00,0x00,0x00,0x00};



static void shelltask( void *pvParameters )
{
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