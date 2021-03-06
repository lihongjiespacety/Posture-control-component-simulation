/**
 * Build this example on linux with:
 * ./waf configure --enable-examples --enable-if-kiss --with-driver-usart=linux --enable-crc32 clean build
 */

#include <stdio.h>
#include <csp/csp.h>
#include <csp/interfaces/csp_if_kiss.h>

#include <csp/drivers/usart.h>
#include <csp/arch/csp_thread.h>
#include "csp_if_can.h"
#include "can.h"
#include "osapi_freertos.h"

#define PORT 10
#define MY_ADDRESS 2
#define SERVER_ADDRESS 2

#define SERVER_TIDX 0
#define CLIENT_TIDX 1
#define USART_HANDLE 0


	static csp_iface_t csp_if_kiss;
	static csp_kiss_handle_t csp_kiss_driver;
        
CSP_DEFINE_TASK(task_server) {
    //int running = 1;
    csp_socket_t *socket = csp_socket(CSP_SO_NONE);
    csp_conn_t *conn;
    csp_packet_t *packet;
    csp_packet_t *response;

    response = csp_buffer_get(sizeof(csp_packet_t) + 2);
    if( response == NULL ) {
        printf("Could not allocate memory for response packet!\n");
        return CSP_TASK_RETURN;
    }
    response->data[0] = 'O';
    response->data[1] = 'K';
    response->length = 2;

    csp_bind(socket, CSP_ANY);
    csp_listen(socket, 5);

    printf("Server task started\r\n");

    while(1) {
        if( (conn = csp_accept(socket, 10000)) == NULL ) {
            continue;
        }

        while( (packet = csp_read(conn, 100)) != NULL ) {
            switch( csp_conn_dport(conn) ) {
                case PORT:
                    if( packet->data[0] == 'q' )
                       // running = 0;
                    csp_send(conn, response, 1000);
                    break;
                default:
                    csp_service_handler(conn, packet);
                    break;
            }
            csp_buffer_free(packet);  /*������ͷ�*/
            packet = NULL;
        }

        csp_close(conn);
    }

    //csp_buffer_free(response);

    //return CSP_TASK_RETURN;
}

CSP_DEFINE_TASK(task_client) {

    char outbuf[] = "1234567890";
    char inbuf[3] = {0};
    //int pingResult;

//    for(int i = 0; i <= 30; i++) {
//      pingResult = csp_ping(SERVER_ADDRESS, 1000, 200, CSP_O_NONE);
//      OsPrintf(OSAPI_DEBUG_INFO,"Ping with payload of %d bytes, took %d ms\r\n", 200, pingResult);
//      csp_sleep_ms(3000);
//    }
//    for(int i = 0; i <= 30; i++) {
//      csp_ps(SERVER_ADDRESS, 1000);
//      csp_sleep_ms(1000);
//    }
//    for(int i = 0; i <= 30; i++) {
//      csp_memfree(SERVER_ADDRESS, 1000);
//      csp_sleep_ms(1000);
//    }
//    for(int i = 0; i <= 30; i++) {
//      csp_buf_free(SERVER_ADDRESS, 1000);
//      csp_sleep_ms(1000);
//    }
//    for(int i = 0; i <= 30; i++) {
//      csp_uptime(SERVER_ADDRESS, 1000);
//      csp_sleep_ms(1000);
//    }
    csp_conn_t * conn = csp_connect(0, SERVER_ADDRESS, PORT, 0, CSP_CONNECTION_SO);
    while(1)
    {
        //csp_transaction(0, SERVER_ADDRESS, PORT, 3000, &outbuf, 1, inbuf, 2);
        int status = csp_transaction_persistent(conn, 2000, outbuf, 10, inbuf, 2);
        printf("Quit response from server: %s\n", inbuf);
        csp_sleep_ms(1000);
    }
    //return CSP_TASK_RETURN;
}

void my_usart_rx(uint8_t * buf, int len, void * pxTaskWoken) 
{
  csp_kiss_rx(&csp_if_kiss, buf, len, pxTaskWoken);
}

int kiss_set(int argc) 
{
  struct csp_can_config canconf;
  canconf.bitrate = (uint32_t)500000;
  canconf.clock_speed = 0;
  canconf.ifc=0;
    csp_debug_toggle_level(CSP_PACKET);
    csp_debug_toggle_level(CSP_INFO);

    csp_buffer_init(10, 300);
    csp_init(MY_ADDRESS);

    struct usart_conf conf;


    /* Run USART init */
    usart_init(&conf);

    /* Setup CSP interface */

    csp_kiss_init(&csp_if_kiss, &csp_kiss_driver, usart_putc, usart_insert, "KISS");
    csp_can_init(CSP_CAN_MASKED, &canconf);	//    CSP_CAN_PROMISC
    /* Setup callback from USART RX to KISS RS */
    usart_set_callback(my_usart_rx);
    csp_route_set(2, &csp_if_can, CSP_NODE_MAC);
    csp_route_set(1, &csp_if_can, CSP_NODE_MAC);
    //csp_route_set(MY_ADDRESS, &csp_if_kiss, CSP_NODE_MAC);
    csp_route_start_task(1024, 3);
#if CSP_DEBUG
    csp_conn_print_table();
    csp_route_print_table();
    csp_route_print_interfaces();
#endif
    if(argc)
    {
      csp_thread_handle_t handle_server;
      csp_thread_create(task_server, "SERVER", 1024, NULL, 3, &handle_server);
    }
    else
    {
      csp_thread_handle_t handle_client;
      csp_thread_create(task_client, "CLIENT", 1024+1024, NULL, 0, &handle_client);
    }
    /* Wait for program to terminate (ctrl + c) */
//    while(1) {
//    	csp_sleep_ms(1000000);
//    }
    return 0;
}
