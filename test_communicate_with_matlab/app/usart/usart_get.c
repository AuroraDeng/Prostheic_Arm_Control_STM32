#include "usart_get.h"
#include <math.h>

uint32_t Get_USART_Command(UART_HandleTypeDef * huart)
{
		u8 len;	
		u32 data=0;
		int i;
	
		len=USART1_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ��ȣ�USART1_RX_STA��0-13λ������ʾ�������ݵĳ���
		for(i=len-1;i>=0;i--)
		{
			data+=pow(10,(len-1-i))*(USART1_RX_BUF[i]-48);
		}	
//		printf("\r\n�����͵���ϢΪ:\r\n");
//		HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART1_RX_BUF,len,1000);	//���ͽ��յ�������		
		print_str(huart,"Receive Command:");
		print_data_dec(huart,data);
		while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)!=SET);		//�ȴ����ͽ���
		printf("\r\n\r\n");//���뻻��
		USART1_RX_STA=0;		
		return data;
}