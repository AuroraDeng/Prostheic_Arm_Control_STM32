#include "usart_host_communication.h"

void host_communicate_slave_test()
{
		if(aRxBuffer[0]==1)
		{
			println_str(&UART1_Handler,"Hello MATLAB");
			while(__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_TC)!=SET);		//µÈ´ý·¢ËÍ½áÊø
			USART1_RX_STA=0;	
			aRxBuffer[0]=0;	
		}
}

