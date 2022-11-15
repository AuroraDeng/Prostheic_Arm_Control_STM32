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
    sprintf(str,"%d ",data);//��ӡ���ַ�����
    HAL_UART_Transmit(huart,(uint8_t*)str,strlen(str),100);
		print_str(huart,"\n");
}

void print_fdata_dec(UART_HandleTypeDef *huart,float data)
{
    char str[25];
    sprintf(str,"%f ",data);//��ӡ���ַ�����
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

//�ô���2��BWTģ�鷢��ָ��
void SendCmdtoBWT(char cmd[])
{
	print_str(&UART2_Handler,(char*)cmd);
}

//void CopeSerial1Data(unsigned char ucData)
//{	
//	UART2_Put_Char(ucData);//ת������1�յ������ݸ�����2��JYģ�飩
//}

//void UART2_Put_Char(unsigned char DataToSend)
//{
//	TxBuffer[count++] = DataToSend;  
//  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);  
//}

//void UART2_Put_String(unsigned char *Str)
//{
//	while(*Str)
//	{
//		if(*Str=='\r')UART2_Put_Char(0x0d);
//			else if(*Str=='\n')UART2_Put_Char(0x0a);
//				else UART2_Put_Char(*Str);
//		Str++;
//	}
//}
//printCANFrame
void printCANframe(u16 id,u8 RTR,u32 len,u8* buff)
{
	int i;
  print_data_hex(&UART1_Handler,id);//ʮ�����ƴ�ӡ����
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


