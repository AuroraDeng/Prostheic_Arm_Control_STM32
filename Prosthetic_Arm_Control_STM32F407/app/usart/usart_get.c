#include "usart_get.h"
#include <math.h>

uint32_t Get_USART_Command(UART_HandleTypeDef * huart)
{
		u8 len;	
		u32 data=0;
		int i;
	
		len=USART1_RX_STA&0x3fff;//得到此次接收到的数据长度：USART1_RX_STA的0-13位用来表示接收数据的长度
		for(i=len-1;i>=0;i--)
		{
			data+=pow(10,(len-1-i))*(USART1_RX_BUF[i]-48);
		}	
//		printf("\r\n您发送的消息为:\r\n");
//		HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART1_RX_BUF,len,1000);	//发送接收到的数据		
		print_str(huart,"Receive Command:");
		print_data_dec(huart,data);
		while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)!=SET);		//等待发送结束
		printf("\r\n\r\n");//插入换行
		USART1_RX_STA=0;		
		return data;
}