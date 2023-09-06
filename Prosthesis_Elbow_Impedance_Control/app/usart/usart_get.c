#include "usart_get.h"
#include <math.h>

void Get_USART_Command(UART_HandleTypeDef * huart,int32_t Command[])
{
		u16 len;	
		int i,j=13;
		u8 num=0;
		
		len=USART1_RX_STA&0x3fff;//得到此次接收到的数据长度：USART1_RX_STA的0-13位用来表示接收数据的长度
		for(i=len-1;i>=0;i--)
		{
			if(USART1_RX_BUF[i]!=' ')
			{
				if(USART1_RX_BUF[i]!='-')
					Command[j]+=pow(10,(len-1-i-num))*(USART1_RX_BUF[i]-48);
				else
					Command[j]=-Command[j];
			}
			else
			{
				j--;
				num=len-i;
			}
		}	
//		printf("\r\n您发送的消息为:\r\n");
//		HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART1_RX_BUF,len,1000);	//发送接收到的数据		
		print_str(huart,"Receive Command:");
		for(i=0;i<14;i++)
			print_data_dec(huart,Command[i]);
		
		while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)!=SET);		//等待发送结束
		printf("\r\n\r\n");//插入换行
		USART1_RX_STA=0;		
}

//void Get_USART_Command(UART_HandleTypeDef * huart,float Command[])
//{
//	u16 len;	
//	int i,j=3;//j:第j个运动控制指令；i:移动读取位置
//	u8 num=0,point=0;//point:记录运动指令中小数点的位置;num=记录前一条运动指令所占的位数
//		
//	len=USART1_RX_STA&0x3fff;//得到此次接收到的数据长度：USART1_RX_STA的0-13位用来表示接收数据的长度
//		for(i=len-1;i>=0;i--)
//		{
//			if(USART1_RX_BUF[i]!=' ')
//			{
//				if(USART1_RX_BUF[i]!='-'&&USART1_RX_BUF[i]!='.')
//					Command[j]+=pow(10,(len-1-i-num-point))*(USART1_RX_BUF[i]-48);
//				else if(USART1_RX_BUF[i]=='-')
//					Command[j]=-Command[j];
//				else
//				{
//					point=(len-1)-i-num;
//					Command[j]=Command[j]*pow(10,-point);
//					point++;
//				}
//			}
//			else
//			{
//				j--;
//				num=len-i;
//				point=0;
//			}
//		}	
////		printf("\r\n您发送的消息为:\r\n");
////		HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART1_RX_BUF,len,1000);	//发送接收到的数据		
//		print_str(huart,"Receive Command:");
//		for(i=0;i<4;i++)
//			print_fdata_dec(huart,Command[i]);
//		
//		while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)!=SET);		//等待发送结束
//		printf("\r\n\r\n");//插入换行
//		USART1_RX_STA=0;		
//}
