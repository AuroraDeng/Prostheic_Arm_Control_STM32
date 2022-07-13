#include "main.h"
#include "stm32f4xx_hal.h"

#include "usart_get.h"
#include "motioncontrol.h"
#include "motor_test.h"
#include "usart_host_communication.h"
int main(void)
{
	u8 len;	
	u16 times=0;
	u32 commandword;
  HAL_Init();                    	//初始化HAL库    
  Stm32_Clock_Init(336,8,2,7);  	//设置时钟,168Mhz
	delay_init(168);               	//初始化延时函数
	Serialport_Init(115200,115200);              //初始化USART
	CAN_Config();

  while(1)
  {
	if(USART1_RX_STA&0x8000)//判断接收是否完成：0x8000=1000 0000 0000 0000/uint16_t USART1_RX_STA的bit15（接收完成标志）置1
			{	
				commandword=Get_USART_Command(&UART1_Handler);	 				   

				switch(commandword)
				{
					case 1:
					{
						test(72000);
						break;
					}
					case 2:
					{
					}
					case 3:
					{
					
					}
				
				}
			}
			else
			{
				times++;
				if(times%200==0)
					println_str(&UART1_Handler,"Please Enter the Prosthetic Arm Movement Command");
				delay_ms(10);   
			} 
//	Message m;
//	m.COB_ID=0x01;
//	m.RTR=0;
//	m.len=0x04;
//	m.Data[0]=0x01;
//	m.Data[1]=0x02;
//	m.Data[2]=0x03;
//	m.Data[3]=0x04;
//	CAN_SendMsg(&m);
  }
}

