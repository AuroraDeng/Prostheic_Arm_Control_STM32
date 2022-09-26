#ifndef _USART_GET_H
#define _USART_GET_H

#include <string.h>
#include <stdio.h>
#include "main.h"
#include "usart.h"
#include "usart_print.h"

#ifdef __cplusplus
 extern "C" {
#endif

void Get_USART_Command(UART_HandleTypeDef * huart,u32 Command[]);	 
	 
#ifdef __cplusplus
}
#endif

#endif
