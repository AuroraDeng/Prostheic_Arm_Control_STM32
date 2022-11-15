#include "usart_get.h"
#include <math.h>

//void Get_USART_Command(UART_HandleTypeDef * huart,int32_t Command[])
//{
//		u16 len;	
//		int i,j=2;
//		u8 num=0;
//		
//		len=USART1_RX_STA&0x3fff;//得到此次接收到的数据长度：USART1_RX_STA的0-13位用来表示接收数据的长度
//		for(i=len-1;i>=0;i--)
//		{
//			if(USART1_RX_BUF[i]!=' ')
//			{
//				if(USART1_RX_BUF[i]!='-')
//					Command[j]+=pow(10,(len-1-i-num))*(USART1_RX_BUF[i]-48);
//				else
//					Command[j]=-Command[j];
//			}
//			else
//			{
//				j--;
//				num=len-i;
//			}
//		}	
////		printf("\r\n您发送的消息为:\r\n");
////		HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART1_RX_BUF,len,1000);	//发送接收到的数据		
//		print_str(huart,"Receive Command:");
//		for(i=0;i<3;i++)
//			print_data_dec(huart,Command[i]);
//		
//		while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)!=SET);		//等待发送结束
//		printf("\r\n\r\n");//插入换行
//		USART1_RX_STA=0;		
//}

void Get_USART_Command(UART_HandleTypeDef * huart,float Command[])
{
		u16 len;	
		int i,j=2;
		u8 num=0,point=0;
		
		len=USART1_RX_STA&0x3fff;//得到此次接收到的数据长度：USART1_RX_STA的0-13位用来表示接收数据的长度
		for(i=len-1;i>=0;i--)
		{
			if(USART1_RX_BUF[i]!=' ')
			{
				if(USART1_RX_BUF[i]!='-'&&USART1_RX_BUF[i]!='.')
					Command[j]+=pow(10,(len-1-i-num-point))*(USART1_RX_BUF[i]-48);
				else if(USART1_RX_BUF[i]=='-')
					Command[j]=-Command[j];
				else
				{
					Command[j]=Command[j]*pow(10,i-1-num);
					point=2+num-i;//记录运动指令中小数点的位置
				}
			}
			else
			{
				j--;
				num=len-i;
				point=0;
			}
		}	
//		printf("\r\n您发送的消息为:\r\n");
//		HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART1_RX_BUF,len,1000);	//发送接收到的数据		
		print_str(huart,"Receive Command:");
		for(i=0;i<3;i++)
			print_fdata_dec(huart,Command[i]);
		
		while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)!=SET);		//等待发送结束
		printf("\r\n\r\n");//插入换行
		USART1_RX_STA=0;		
}

