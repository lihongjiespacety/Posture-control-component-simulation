

#include <stdio.h>
#include <time.h>
#include <string.h>
#include <ctype.h>
#include "shell_fun.h"
#include "hal.h"
#include "driver.h"
#include "osapi_freertos.h"
#include "shell.h"
#include "clock.h"
#include "stm32f10x.h"

timestamp_t g_clock_t = {(uint32_t)0,(uint32_t)0};

void vApplicationTickHook()
{
  ///__disable_interrupt();   中断中调用
    g_clock_t.tv_usec += (uint32_t)1000000 / configTICK_RATE_HZ;
//    if((g_clock_t.tv_usec % (uint32_t)1000000) == 0)
//    {
//      GPIOD->ODR ^= 1<<13;
//    }
    while(g_clock_t.tv_usec >= (uint32_t)1000000)
    {
      GPIOD->ODR ^= 1<<13;
      g_clock_t.tv_usec -= (uint32_t)1000000;
      g_clock_t.tv_sec++;
      driver_time_sechandle();
    }
  ///__enable_interrupt(); 
}


void clock_get_time(timestamp_t * time)
{
  __disable_interrupt();   
  time->tv_usec = g_clock_t.tv_usec;
  time->tv_sec =  g_clock_t.tv_sec;
  __enable_interrupt(); 
}

void clock_set_time(timestamp_t * time)
{
  __disable_interrupt();     
  g_clock_t.tv_usec = time->tv_usec;
  g_clock_t.tv_sec = time->tv_sec;
  __enable_interrupt(); 
}