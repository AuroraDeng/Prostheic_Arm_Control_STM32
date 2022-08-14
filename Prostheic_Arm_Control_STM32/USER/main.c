#include "main.h"
#include "stm32f4xx_hal.h"

#include "usart_get.h"
#include "motioncontrol.h"
#include "motor_test.h"
#include "usart_host_communication.h"
int main(void)
{
	Message m;
	u16 times=0;
	u32 commandword;
  HAL_Init();                    	//��ʼ��HAL��    
  Stm32_Clock_Init(336,8,2,7);  	//����ʱ��,168Mhz
	delay_init(168);               	//��ʼ����ʱ����
	Serialport_Init(115200,115200);              //��ʼ��USART
	CAN_Config();

  while(1)
  {
			if(USART1_RX_STA&0x8000)//�жϽ����Ƿ���ɣ�0x8000=1000 0000 0000 0000/uint16_t USART1_RX_STA��bit15��������ɱ�־����1
			{	
				commandword=Get_USART_Command(&UART1_Handler);	 				   

				switch(commandword)
				{
					case 1:
					{
						Wrist_Extension();
						break;
					}
					case 2:
					{
						Wrist_Flextion();
						break;
					}
					case 3:
					{
						Wrist_Ulnar_Deviation();
						break;
					}
					case 4:
					{
						Wrist_Radial_Deviation();
						break;
					}
					case 5:
					{
						HelloWorld();
						break;
					}
					case 6:
					{
						Motor_Reset();
						break;
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
  }
}

