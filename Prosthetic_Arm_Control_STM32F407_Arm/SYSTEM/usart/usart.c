#include "usart.h"
#include "delay.h"

////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��os,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//os ʹ��	  
#endif
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	

//�ض���fputc����
int fputc(int ch,FILE *f)
{
//    uint8_t temp[1]={ch};
//    if(HAL_UART_Transmit(&UART1_Handler,temp,1,2))
//			return 1;
//		else
//			return 0;
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}

#if EN_USART1_RX   //���ʹ���˴���1����
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART1_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.

//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART1_RX_STA=0;       //����״̬���	

u8 aRxBuffer[RXBUFFERSIZE];//HAL��ʹ�õĴ��ڽ��ջ���

UART_HandleTypeDef UART1_Handler; //UART���

//��ʼ��IO ����1 
//bound:������
void uart1_init(u32 bound)
{	
	//UART ��ʼ������
	UART1_Handler.Instance=USART1;					    //USART1
	UART1_Handler.Init.BaudRate=bound;				    //������
	UART1_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	UART1_Handler.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	UART1_Handler.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	UART1_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	UART1_Handler.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&UART1_Handler);					    //HAL_UART_Init()��ʹ��UART1
	
	HAL_UART_Receive_IT(&UART1_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
  
}

//UART�ײ��ʼ����ʱ��ʹ�ܣ��������ã��ж�����
//�˺����ᱻHAL_UART_Init()����
//huart:���ھ��

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO�˿�����
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance==USART1)//����Ǵ���1�����д���1 MSP��ʼ��
	{
		/**USART3 GPIO Configuration
    PA9     ------> USART3_TX
    PA10     ------> USART3_RX
    */
		__HAL_RCC_GPIOA_CLK_ENABLE();			//ʹ��GPIOAʱ��
		__HAL_RCC_USART1_CLK_ENABLE();			//ʹ��USART1ʱ��
	
		GPIO_Initure.Pin=GPIO_PIN_9;			//PA9
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//����
		GPIO_Initure.Alternate=GPIO_AF7_USART1;	//����ΪUSART1
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA9

		GPIO_Initure.Pin=GPIO_PIN_10;			//PA10
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA10
		
#if EN_USART1_RX
		HAL_NVIC_EnableIRQ(USART1_IRQn);				//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(USART1_IRQn,3,3);			//��ռ���ȼ�3�������ȼ�3
#endif	
	}
	
	if(huart->Instance==USART2)//����Ǵ���2�����д���2 MSP��ʼ��
	{
		/**USART3 GPIO Configuration
    PA2     ------> USART3_TX
    PA3     ------> USART3_RX
    */
		__HAL_RCC_GPIOA_CLK_ENABLE();			//ʹ��GPIOAʱ��
		__HAL_RCC_USART2_CLK_ENABLE();			//ʹ��USART1ʱ��
	
		GPIO_Initure.Pin=GPIO_PIN_2;			//PA2
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//����
		GPIO_Initure.Alternate=GPIO_AF7_USART2;	//����ΪUSART2
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA2

		GPIO_Initure.Pin=GPIO_PIN_3;			//PA3
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA3
		
#if EN_USART2_RX
		HAL_NVIC_EnableIRQ(USART2_IRQn);				//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(USART2_IRQn,3,3);			//��ռ���ȼ�3�������ȼ�3
#endif	
	}
	
	if(huart->Instance==USART3)//����Ǵ���3�����д���3 MSP��ʼ��
	{ /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();			//ʹ��USART3ʱ��
    __HAL_RCC_GPIOB_CLK_ENABLE();				//ʹ��GPIOBʱ��
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
		GPIO_Initure.Pin=GPIO_PIN_10;			//PB10
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//����
		GPIO_Initure.Alternate=GPIO_AF7_USART3;	//����ΪUSART3
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//��ʼ��PB10

		GPIO_Initure.Pin=GPIO_PIN_11;			//PB11
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//��ʼ��PB11
		
#if EN_USART3_RX
		HAL_NVIC_EnableIRQ(USART3_IRQn);				//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(USART3_IRQn,3,3);			//��ռ���ȼ�3�������ȼ�3
#endif	
	}
	
	if(huart->Instance==UART4)//����4MSP��ʼ��
	{
		/*
			UART4_TX --> GPIOC Pin 10
			UART4_RX --> GPIOC Pin 11
		*/
		__HAL_RCC_GPIOC_CLK_ENABLE();			//ʹ��GPIOCʱ��
		__HAL_RCC_UART4_CLK_ENABLE();			//ʹ��UART4ʱ��
	
		GPIO_Initure.Pin=GPIO_PIN_10|GPIO_PIN_11;			//PC10��PC11
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//����
		GPIO_Initure.Alternate=GPIO_AF8_UART4;	//����ΪUART4
		HAL_GPIO_Init(GPIOC,&GPIO_Initure);	   	//��ʼ��PA10��PA11
		
#if EN_UART4_RX
		HAL_NVIC_EnableIRQ(UART4_IRQn);				//ʹ��UART4�ж�ͨ��
		HAL_NVIC_SetPriority(UART4_IRQn,3,3);			//��ռ���ȼ�1�������ȼ�8
#endif	
	}
		
	if(huart->Instance==UART5)//����Ǵ���5�����д���5 MSP��ʼ��
	{
		/*
			USART6_TX --> GPIOB Pin 8
			USART6_RX --> GPIOB Pin 9
		*/
		__HAL_RCC_GPIOB_CLK_ENABLE();			//ʹ��GPIOBʱ��
		__HAL_RCC_UART5_CLK_ENABLE();			//ʹ��UART2ʱ��
	
		GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9;			//PB8/PB9
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//����
		GPIO_Initure.Alternate=GPIO_AF8_UART5;	//����ΪUART5
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//��ʼ��GPIO
		
#if EN_UART5_RX
		HAL_NVIC_EnableIRQ(UART5_IRQn);				//ʹ��UART5�ж�ͨ��
		HAL_NVIC_SetPriority(UART5_IRQn,3,3);			//��ռ���ȼ�3�������ȼ�3
#endif	
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)//����Ǵ���1
	{
		if((USART1_RX_STA&0x8000)==0)//����δ��ɣ�0x8000=1000 0000 0000 0000/uint16_t USART1_RX_STA��bit15��������ɱ�־����1
		{
			if(USART1_RX_STA&0x4000)//���յ���0x0d��0x4000=0100 0000 0000 0000/uint16_t USART1_RX_STA��bit14�����յ�0x0d��־����1/���յ����س�������
			{
				if(aRxBuffer[0]!=0x0a)USART1_RX_STA=0;//û���յ�0x0a�����з������մ���,���¿�ʼ
				else USART1_RX_STA|=0x8000;	//��������� 
			}
			else //��û�յ�0X0D
			{	
				if(aRxBuffer[0]==0x0d)USART1_RX_STA|=0x4000;//���յ�0x0d�����յ����س������ţ�uint16_t USART1_RX_STA��bit14�����յ�0x0d��־����1
				else
				{
					USART1_RX_BUF[USART1_RX_STA&0X3FFF]=aRxBuffer[0] ;//�����ջ��������ÿ���ֽ�һһ���浽����������
					USART1_RX_STA++;
					if(USART1_RX_STA>(USART_REC_LEN-1))USART1_RX_STA=0;//�������ֽ������ڽ�������������������Ϊ���ݴ���,���¿�ʼ����	  
				}		 
			}
		}
	}
	else if(huart->Instance==USART2)//����Ǵ���2
////		CopeWristPosData((unsigned char)bRxBuffer[0]);//��������
			CopeMPData((unsigned char)bRxBuffer[0]);
	else if(huart->Instance==UART4)
		CopeSPData((unsigned char)dRxBuffer[0]);
	else if(huart->Instance==UART5)
		WitSerialDataIn((unsigned char)eRxBuffer[0]);
//	else if(huart->Instance==USART3)//����Ǵ���3
////		CopeElbowFTData((unsigned char *)cRxBuffer,wCount);	 
//		CopeElbowFTData((unsigned char)cRxBuffer,wCount);
}
 
//����1�жϷ������
void USART1_IRQHandler(void)                	
{ 
	u32 timeout=0;
	
	HAL_UART_IRQHandler(&UART1_Handler);	//����HAL���жϴ����ú���
	
	timeout=0;
    while (HAL_UART_GetState(&UART1_Handler) != HAL_UART_STATE_READY)//�ȴ�����
	{
	 timeout++;////��ʱ����
     if(timeout>HAL_MAX_DELAY) break;		
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&UART1_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)//һ�δ������֮�����¿����жϲ�����RxXferCountΪ1
	{
	 timeout++; //��ʱ����
	 if(timeout>HAL_MAX_DELAY) break;	
	}
} 
#endif	

#if EN_USART2_RX   //���ʹ���˴���2����
//u8 USART2_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
u16 USART2_RX_STA=0;       //����״̬���	
u8 bRxBuffer[RXBUFFERSIZE];
UART_HandleTypeDef UART2_Handler; //UART���

//��ʼ��IO ����1 
//bound:������
void uart2_init(u32 bound)
{	
	//UART ��ʼ������
	UART2_Handler.Instance=USART2;					    //USART1
	UART2_Handler.Init.BaudRate=bound;				    //������
	UART2_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	UART2_Handler.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	UART2_Handler.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	UART2_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	UART2_Handler.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&UART2_Handler);					    //HAL_UART_Init()��ʹ��UART2
	
	HAL_UART_Receive_IT(&UART2_Handler, (u8 *)bRxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
}

//����2�жϷ������
void USART2_IRQHandler(void)                	
{ 	
	u32 timeout=0;
	
	HAL_UART_IRQHandler(&UART2_Handler);	//����HAL���жϴ����ú���
	
  while (HAL_UART_GetState(&UART2_Handler) != HAL_UART_STATE_READY)//�ȴ�����
	{
		timeout++;////��ʱ����
		if(timeout>HAL_MAX_DELAY) break;		
	}
	
	timeout=0;
	while(HAL_UART_Receive_IT(&UART2_Handler, (u8 *)bRxBuffer, RXBUFFERSIZE) != HAL_OK)//һ�δ������֮�����¿����жϲ�����RxXferCountΪ1
	{
	 timeout++; //��ʱ����
	 if(timeout>HAL_MAX_DELAY) break;	
	}
	
	__HAL_UART_CLEAR_FLAG(&UART2_Handler,UART_FLAG_RXNE);
	__HAL_UART_CLEAR_FLAG(&UART2_Handler,UART_FLAG_ORE);
} 
#endif	

#if EN_USART3_RX

//u8 USART3_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
u16 USART3_RX_STA=0;       //����״̬���	
u8 cRxBuffer[200];
UART_HandleTypeDef UART3_Handler; //UART���

//��ʼ��IO ����1 
//bound:������
void uart3_init(u32 bound)
{	
	//UART ��ʼ������
	UART3_Handler.Instance=USART3;					    //USART3
	UART3_Handler.Init.BaudRate=bound;				    //������
	UART3_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	UART3_Handler.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	UART3_Handler.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	UART3_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	UART3_Handler.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&UART3_Handler);					    //HAL_UART_Init()��ʹ��UART3
	
	 /* �޸Ľ��ջ�������С */
  UART3_Handler.pRxBuffPtr = cRxBuffer;
  UART3_Handler.RxXferSize = sizeof(cRxBuffer);
  UART3_Handler.RxXferCount = sizeof(cRxBuffer);
}
#endif

#if EN_UART4_RX
u8 dRxBuffer[RXBUFFERSIZE];
UART_HandleTypeDef UART4_Handler; //UART���

//��ʼ��IO ����1 
//bound:������
void uart4_init(u32 bound)
{	
	//UART ��ʼ������
	UART4_Handler.Instance=UART4;					    //UART4
	UART4_Handler.Init.BaudRate=bound;				    //������
	UART4_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	UART4_Handler.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	UART4_Handler.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	UART4_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	UART4_Handler.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&UART4_Handler);					    //HAL_UART_Init()��ʹ��UART4
	
	HAL_UART_Receive_IT(&UART4_Handler, (u8 *)dRxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
}

//����4�жϷ������
void UART4_IRQHandler(void)                	
{ 	
	u32 timeout=0;
	
	HAL_UART_IRQHandler(&UART4_Handler);	//����HAL���жϴ����ú���
	
  while (HAL_UART_GetState(&UART4_Handler) != HAL_UART_STATE_READY)//�ȴ�����
	{
		timeout++;////��ʱ����
		if(timeout>HAL_MAX_DELAY) break;		
	}
	
	timeout=0;
	while(HAL_UART_Receive_IT(&UART4_Handler, (u8 *)dRxBuffer, RXBUFFERSIZE) != HAL_OK)//һ�δ������֮�����¿����жϲ�����RxXferCountΪ1
	{
	 timeout++; //��ʱ����
	 if(timeout>HAL_MAX_DELAY) break;	
	}
	
	__HAL_UART_CLEAR_FLAG(&UART4_Handler,UART_FLAG_RXNE);
	__HAL_UART_CLEAR_FLAG(&UART4_Handler,UART_FLAG_ORE);
} 

#endif

#if EN_UART5_RX   //���ʹ���˴���2����
u8 eRxBuffer[RXBUFFERSIZE];
UART_HandleTypeDef UART5_Handler; //UART���

//��ʼ��IO ����5 
//bound:������
void uart5_init(u32 bound)
{	
	//UART ��ʼ������
	UART5_Handler.Instance=UART5;					    //USART5
	UART5_Handler.Init.BaudRate=bound;				    //������
	UART5_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	UART5_Handler.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	UART5_Handler.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	UART5_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	UART5_Handler.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&UART5_Handler);					    //HAL_UART_Init()��ʹ��UART2
	
	HAL_UART_Receive_IT(&UART5_Handler, (u8 *)eRxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
}

//����5�жϷ������
void UART5_IRQHandler(void)                	
{ 	
	u32 timeout=0;
	
	HAL_UART_IRQHandler(&UART5_Handler);	//����HAL���жϴ����ú���
	
  while (HAL_UART_GetState(&UART5_Handler) != HAL_UART_STATE_READY)//�ȴ�����
	{
		timeout++;////��ʱ����
		if(timeout>HAL_MAX_DELAY) break;		
	}
	
	timeout=0;
	while(HAL_UART_Receive_IT(&UART5_Handler, (u8 *)eRxBuffer, RXBUFFERSIZE) != HAL_OK)//һ�δ������֮�����¿����жϲ�����RxXferCountΪ1
	{
	 timeout++; //��ʱ����
	 if(timeout>HAL_MAX_DELAY) break;	
	}
	
	__HAL_UART_CLEAR_FLAG(&UART5_Handler,UART_FLAG_RXNE);
	__HAL_UART_CLEAR_FLAG(&UART5_Handler,UART_FLAG_ORE);
} 
#endif	

void Serialport_Init(u32 bound1,u32 bound2,u32 bound3,u32 bound4,u32 bound5)
{
	uart1_init(bound1);
	uart2_init(bound2);
	uart3_init(bound3);
	uart4_init(bound4);
	uart5_init(bound5);
}




