#ifndef _USART_PRINT_H
#define _USART_PRINT_H

#include <string.h>
#include <stdio.h>
#include "main.h"
#include "usart.h"

#ifdef __cplusplus
 extern "C" {
#endif
	 
//wrapper
void print_str(UART_HandleTypeDef * huart,const char *str);
void print_data_hex(UART_HandleTypeDef *huart,uint32_t data);
void print_data_dec(UART_HandleTypeDef *huart,uint32_t data);
void print_data_format(UART_HandleTypeDef *huart,uint32_t data, char * format);
void println_str(UART_HandleTypeDef *huart,const char *str);
void usart1_send_string (uint8_t *str);
void printCANframe(u16 id,u8 RTR,u32 len,u8* buff);

#ifdef __cplusplus
}
#endif

#endif

