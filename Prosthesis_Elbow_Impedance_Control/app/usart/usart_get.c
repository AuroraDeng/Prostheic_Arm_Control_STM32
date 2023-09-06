#include "usart_get.h"
#include <math.h>

void Get_USART_Command(UART_HandleTypeDef * huart,int32_t Command[])
{
		u16 len;	
		int i,j=13;
		u8 num=0;
		
		len=USART1_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ��ȣ�USART1_RX_STA��0-13λ������ʾ�������ݵĳ���
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
//		printf("\r\n�����͵���ϢΪ:\r\n");
//		HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART1_RX_BUF,len,1000);	//���ͽ��յ�������		
		print_str(huart,"Receive Command:");
		for(i=0;i<14;i++)
			print_data_dec(huart,Command[i]);
		
		while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)!=SET);		//�ȴ����ͽ���
		printf("\r\n\r\n");//���뻻��
		USART1_RX_STA=0;		
}

//void Get_USART_Command(UART_HandleTypeDef * huart,float Command[])
//{
//	u16 len;	
//	int i,j=3;//j:��j���˶�����ָ�i:�ƶ���ȡλ��
//	u8 num=0,point=0;//point:��¼�˶�ָ����С�����λ��;num=��¼ǰһ���˶�ָ����ռ��λ��
//		
//	len=USART1_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ��ȣ�USART1_RX_STA��0-13λ������ʾ�������ݵĳ���
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
////		printf("\r\n�����͵���ϢΪ:\r\n");
////		HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART1_RX_BUF,len,1000);	//���ͽ��յ�������		
//		print_str(huart,"Receive Command:");
//		for(i=0;i<4;i++)
//			print_fdata_dec(huart,Command[i]);
//		
//		while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)!=SET);		//�ȴ����ͽ���
//		printf("\r\n\r\n");//���뻻��
//		USART1_RX_STA=0;		
//}
