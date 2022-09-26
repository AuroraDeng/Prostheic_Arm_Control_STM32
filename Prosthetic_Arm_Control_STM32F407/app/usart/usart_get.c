#include "usart_get.h"
#include <math.h>

void Get_USART_Command(UART_HandleTypeDef * huart,u32 Command[])
{
		u16 len;	
		int i,j=0;
		u8 num=0;
		
		len=USART1_RX_STA&0x3fff;//得到此次接收到的数据长度：USART1_RX_STA的0-13位用来表示接收数据的长度
		for(i=len-1;i>=0;i--)
		{
			if(USART1_RX_BUF[i]!=' ')
				Command[j]+=pow(10,(len-1-i-num))*(USART1_RX_BUF[i]-48);
			else
			{
				j++;
				num=len-i;
			}
		}	
//		printf("\r\n您发送的消息为:\r\n");
//		HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART1_RX_BUF,len,1000);	//发送接收到的数据		
		print_str(huart,"Receive Command:");
		for(i=0;i<3;i++)
		{
			print_data_dec(huart,Command[i]);
			print_str(huart,"\n");
		}
		while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)!=SET);		//等待发送结束
		printf("\r\n\r\n");//插入换行
		USART1_RX_STA=0;		
}