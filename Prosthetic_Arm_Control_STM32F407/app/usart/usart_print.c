#include "usart_print.h"

void print_str(UART_HandleTypeDef * huart,const char *str)
{
    HAL_UART_Transmit(huart,(uint8_t*)str,strlen(str),100);
}

void print_data_hex(UART_HandleTypeDef * huart,uint32_t data)
{
    char str[25];
    sprintf(str,"%x",data);
    HAL_UART_Transmit(huart,(uint8_t*)str,strlen(str),100);
}

void print_data_dec(UART_HandleTypeDef *huart,uint32_t data)
{
    char str[25];
    sprintf(str,"%d ",data);//打印到字符串中
    HAL_UART_Transmit(huart,(uint8_t*)str,strlen(str),100);
		print_str(huart,"\n");
}

void print_data_format(UART_HandleTypeDef *huart,uint32_t data, char * format)
{
    char str[25];
    sprintf(str,format,data);
    HAL_UART_Transmit(huart,(uint8_t*)str,strlen(str),100);
}

void println_str(UART_HandleTypeDef *huart,const char * str)
{
    HAL_UART_Transmit(huart,(uint8_t*)str,strlen(str),100);
    print_str(huart,"\n");
}

void usart1_send_string (uint8_t *str)
{
    print_str(&UART1_Handler,(char*)str);
}

//printCANFrame
void printCANframe(u16 id,u8 RTR,u32 len,u8* buff)
{
	int i;
  print_data_hex(&UART1_Handler,id);//十六进制打印数据
	print_str(&UART1_Handler," ");
	
	print_data_dec(&UART1_Handler,RTR);
	print_str(&UART1_Handler," ");
	
   for(i = 0; i < len; i++)
   {
       print_data_format(&UART1_Handler,buff[i],"%02x");
       print_str(&UART1_Handler," ");
   }
    print_str(&UART1_Handler,"\n");
	 USART1_RX_STA=0;		
}

