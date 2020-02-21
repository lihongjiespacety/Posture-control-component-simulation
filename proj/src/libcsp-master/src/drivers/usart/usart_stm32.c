#include <stdio.h>
#include <csp/csp.h>
#include <csp/drivers/usart.h>
#include "driver_uart.h"

#if defined(EPS)
#define CSP_UART UART_ID_3
#endif
#if defined(PAYLOAD_EPS)
#define CSP_UART UART_ID_5   /*载荷EPS的PD9 PB10作为IO 所以不能使用UART_ID_3*/
#endif 
usart_callback_t  usart_callback = NULL;

void csp_uart_init(void)
{
    uart_cfg_t cfg;
    cfg.id = CSP_UART;
    cfg.baud = UART_BAUD_500000;
    cfg.datalen = UART_DATA_8;
    cfg.parity = UART_CHECK_NONE;
    cfg.stopb =  UART_STOPB_1;
    driver_uart_init(CSP_UART);
    driver_uart_set(&cfg);
    driver_uart_flush(CSP_UART);
}

/*接收任务循环调用接收回调函数*/
unsigned  prvRxTask(void* params) 
{
  
  uint8_t buff[100];
  uint16_t len;
  int8_t erro;
  len = driver_uart_recv(CSP_UART, buff, sizeof(buff), 100, &erro);
  if(len)
  {
    if( usart_callback != NULL )
    {
      usart_callback(buff, len, NULL);
    }
  }
  return 0;
}

void usart_shutdown(void) 
{

}



void usart_putstr(char* buf, int bufsz) 
{
  int8_t erro;
  driver_uart_send(CSP_UART, (unsigned char*)buf, bufsz, 100, &erro);
}

void usart_putc(char c) 
{
  int8_t erro;
  unsigned char tmp=c;
  driver_uart_send(CSP_UART, &tmp, 1, 10, &erro);
}


void usart_insert(char c, void *pxTaskWoken) 
{
    /* redirect debug output to stdout */
    printf("%c", c);
}

void usart_set_callback(usart_callback_t callback) 
{
    usart_callback = callback;
}

void usart_init(struct usart_conf * conf) 
{

}


