#include "usart_get.h"
#include <math.h>

void Get_USART_Command(UART_HandleTypeDef * huart,u32 Command[])
{
		u16 len;	
		int i,j=0;
		u8 num=0;
		
		len=USART1_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ��ȣ�USART1_RX_STA��0-13λ������ʾ�������ݵĳ���
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
//		printf("\r\n�����͵���ϢΪ:\r\n");
//		HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART1_RX_BUF,len,1000);	//���ͽ��յ�������		
		print_str(huart,"Receive Command:");
		for(i=0;i<3;i++)
		{
			print_data_dec(huart,Command[i]);
			print_str(huart,"\n");
		}
		while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)!=SET);		//�ȴ����ͽ���
		printf("\r\n\r\n");//���뻻��
		USART1_RX_STA=0;		
}