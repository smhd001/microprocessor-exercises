/*
 * uart.c
 *
 *  Created on: Jul 5, 2023
 *      Author: mhmmd
 */
#include "main.h"
#include "program.h"
#include "scenario.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>


#define pass 170
unsigned char message[20];
extern UART_HandleTypeDef huart1;
extern int is_on;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1 && is_on) {
    	if(message[0] == pass)
    	{
		Config conf =  {
				message[1],message[2],message[3],message[4],
				{message[6],message[7],message[8],message[9]}};
    		set_scenario(conf);
    	}
        HAL_UART_Receive_IT(&huart1, message, 10);
    }
}
void log_uart ( const char * format, ... )
{
  va_list args;
  va_start (args, format);
  char msg[100];
  vsprintf (msg,format, args);
  HAL_UART_Transmit(&huart1, msg, strlen(msg), 1000);
  va_end (args);
}
