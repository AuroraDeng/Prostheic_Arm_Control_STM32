#include "usart.h"
#include "delay.h"

////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用os,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//os 使用	  
#endif
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	

//重定义fputc函数
int fputc(int ch,FILE *f)
{
//    uint8_t temp[1]={ch};
//    if(HAL_UART_Transmit(&UART1_Handler,temp,1,2))
//			return 1;
//		else
//			return 0;
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}

#if EN_USART1_RX   //如果使能了串口1接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART1_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.

//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART1_RX_STA=0;       //接收状态标记	

u8 aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲

UART_HandleTypeDef UART1_Handler; //UART句柄

//初始化IO 串口1 
//bound:波特率
void uart1_init(u32 bound)
{	
	//UART 初始化设置
	UART1_Handler.Instance=USART1;					    //USART1
	UART1_Handler.Init.BaudRate=bound;				    //波特率
	UART1_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART1_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART1_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART1_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART1_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART1_Handler);					    //HAL_UART_Init()会使能UART1
	
	HAL_UART_Receive_IT(&UART1_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
  
}

//UART底层初始化，时钟使能，引脚配置，中断配置
//此函数会被HAL_UART_Init()调用
//huart:串口句柄

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO端口设置
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance==USART1)//如果是串口1，进行串口1 MSP初始化
	{
		/**USART3 GPIO Configuration
    PA9     ------> USART3_TX
    PA10     ------> USART3_RX
    */
		__HAL_RCC_GPIOA_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_USART1_CLK_ENABLE();			//使能USART1时钟
	
		GPIO_Initure.Pin=GPIO_PIN_9;			//PA9
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//高速
		GPIO_Initure.Alternate=GPIO_AF7_USART1;	//复用为USART1
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA9

		GPIO_Initure.Pin=GPIO_PIN_10;			//PA10
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA10
		
#if EN_USART1_RX
		HAL_NVIC_EnableIRQ(USART1_IRQn);				//使能USART1中断通道
		HAL_NVIC_SetPriority(USART1_IRQn,3,3);			//抢占优先级3，子优先级3
#endif	
	}
	
	if(huart->Instance==USART2)//如果是串口2，进行串口2 MSP初始化
	{
		/**USART3 GPIO Configuration
    PA2     ------> USART3_TX
    PA3     ------> USART3_RX
    */
		__HAL_RCC_GPIOA_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_USART2_CLK_ENABLE();			//使能USART1时钟
	
		GPIO_Initure.Pin=GPIO_PIN_2;			//PA2
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//高速
		GPIO_Initure.Alternate=GPIO_AF7_USART2;	//复用为USART2
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA2

		GPIO_Initure.Pin=GPIO_PIN_3;			//PA3
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA3
		
#if EN_USART2_RX
		HAL_NVIC_EnableIRQ(USART2_IRQn);				//使能USART1中断通道
		HAL_NVIC_SetPriority(USART2_IRQn,3,3);			//抢占优先级3，子优先级3
#endif	
	}
	
	if(huart->Instance==USART3)//如果是串口3，进行串口3 MSP初始化
	{ /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();			//使能USART3时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();				//使能GPIOB时钟
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
		GPIO_Initure.Pin=GPIO_PIN_10;			//PB10
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//高速
		GPIO_Initure.Alternate=GPIO_AF7_USART3;	//复用为USART3
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//初始化PB10

		GPIO_Initure.Pin=GPIO_PIN_11;			//PB11
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//初始化PB11
		
#if EN_USART3_RX
		HAL_NVIC_EnableIRQ(USART3_IRQn);				//使能USART1中断通道
		HAL_NVIC_SetPriority(USART3_IRQn,3,3);			//抢占优先级3，子优先级3
#endif	
	}
	
	if(huart->Instance==UART4)//串口4MSP初始化
	{
		/*
			UART4_TX --> GPIOC Pin 10
			UART4_RX --> GPIOC Pin 11
		*/
		__HAL_RCC_GPIOC_CLK_ENABLE();			//使能GPIOC时钟
		__HAL_RCC_UART4_CLK_ENABLE();			//使能UART4时钟
	
		GPIO_Initure.Pin=GPIO_PIN_10|GPIO_PIN_11;			//PC10、PC11
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//高速
		GPIO_Initure.Alternate=GPIO_AF8_UART4;	//复用为UART4
		HAL_GPIO_Init(GPIOC,&GPIO_Initure);	   	//初始化PA10、PA11
		
#if EN_UART4_RX
		HAL_NVIC_EnableIRQ(UART4_IRQn);				//使能UART4中断通道
		HAL_NVIC_SetPriority(UART4_IRQn,3,3);			//抢占优先级1，子优先级8
#endif	
	}
		
	if(huart->Instance==UART5)//如果是串口5，进行串口5 MSP初始化
	{
		/*
			USART6_TX --> GPIOB Pin 8
			USART6_RX --> GPIOB Pin 9
		*/
		__HAL_RCC_GPIOB_CLK_ENABLE();			//使能GPIOB时钟
		__HAL_RCC_UART5_CLK_ENABLE();			//使能UART2时钟
	
		GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9;			//PB8/PB9
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//高速
		GPIO_Initure.Alternate=GPIO_AF8_UART5;	//复用为UART5
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//初始化GPIO
		
#if EN_UART5_RX
		HAL_NVIC_EnableIRQ(UART5_IRQn);				//使能UART5中断通道
		HAL_NVIC_SetPriority(UART5_IRQn,3,3);			//抢占优先级3，子优先级3
#endif	
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)//如果是串口1
	{
		if((USART1_RX_STA&0x8000)==0)//接收未完成：0x8000=1000 0000 0000 0000/uint16_t USART1_RX_STA的bit15（接收完成标志）置1
		{
			if(USART1_RX_STA&0x4000)//接收到了0x0d：0x4000=0100 0000 0000 0000/uint16_t USART1_RX_STA的bit14（接收到0x0d标志）置1/接收到“回车”符号
			{
				if(aRxBuffer[0]!=0x0a)USART1_RX_STA=0;//没有收到0x0a（换行符）接收错误,重新开始
				else USART1_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(aRxBuffer[0]==0x0d)USART1_RX_STA|=0x4000;//接收到0x0d，接收到“回车”符号，uint16_t USART1_RX_STA的bit14（接收到0x0d标志）置1
				else
				{
					USART1_RX_BUF[USART1_RX_STA&0X3FFF]=aRxBuffer[0] ;//将接收缓冲区里的每个字节一一保存到接收数组中
					USART1_RX_STA++;
					if(USART1_RX_STA>(USART_REC_LEN-1))USART1_RX_STA=0;//当接收字节数大于接收数组的最大容量则认为数据错误,重新开始接收	  
				}		 
			}
		}
	}
	else if(huart->Instance==USART2)//如果是串口2
////		CopeWristPosData((unsigned char)bRxBuffer[0]);//处理数据
			CopeMPData((unsigned char)bRxBuffer[0]);
	else if(huart->Instance==UART4)
		CopeSPData((unsigned char)dRxBuffer[0]);
	else if(huart->Instance==UART5)
		WitSerialDataIn((unsigned char)eRxBuffer[0]);
//	else if(huart->Instance==USART3)//如果是串口3
////		CopeElbowFTData((unsigned char *)cRxBuffer,wCount);	 
//		CopeElbowFTData((unsigned char)cRxBuffer,wCount);
}
 
//串口1中断服务程序
void USART1_IRQHandler(void)                	
{ 
	u32 timeout=0;
	
	HAL_UART_IRQHandler(&UART1_Handler);	//调用HAL库中断处理公用函数
	
	timeout=0;
    while (HAL_UART_GetState(&UART1_Handler) != HAL_UART_STATE_READY)//等待就绪
	{
	 timeout++;////超时处理
     if(timeout>HAL_MAX_DELAY) break;		
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&UART1_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)//一次处理完成之后，重新开启中断并设置RxXferCount为1
	{
	 timeout++; //超时处理
	 if(timeout>HAL_MAX_DELAY) break;	
	}
} 
#endif	

#if EN_USART2_RX   //如果使能了串口2接收
//u8 USART2_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u16 USART2_RX_STA=0;       //接收状态标记	
u8 bRxBuffer[RXBUFFERSIZE];
UART_HandleTypeDef UART2_Handler; //UART句柄

//初始化IO 串口1 
//bound:波特率
void uart2_init(u32 bound)
{	
	//UART 初始化设置
	UART2_Handler.Instance=USART2;					    //USART1
	UART2_Handler.Init.BaudRate=bound;				    //波特率
	UART2_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART2_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART2_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART2_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART2_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART2_Handler);					    //HAL_UART_Init()会使能UART2
	
	HAL_UART_Receive_IT(&UART2_Handler, (u8 *)bRxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
}

//串口2中断服务程序
void USART2_IRQHandler(void)                	
{ 	
	u32 timeout=0;
	
	HAL_UART_IRQHandler(&UART2_Handler);	//调用HAL库中断处理公用函数
	
  while (HAL_UART_GetState(&UART2_Handler) != HAL_UART_STATE_READY)//等待就绪
	{
		timeout++;////超时处理
		if(timeout>HAL_MAX_DELAY) break;		
	}
	
	timeout=0;
	while(HAL_UART_Receive_IT(&UART2_Handler, (u8 *)bRxBuffer, RXBUFFERSIZE) != HAL_OK)//一次处理完成之后，重新开启中断并设置RxXferCount为1
	{
	 timeout++; //超时处理
	 if(timeout>HAL_MAX_DELAY) break;	
	}
	
	__HAL_UART_CLEAR_FLAG(&UART2_Handler,UART_FLAG_RXNE);
	__HAL_UART_CLEAR_FLAG(&UART2_Handler,UART_FLAG_ORE);
} 
#endif	

#if EN_USART3_RX

//u8 USART3_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u16 USART3_RX_STA=0;       //接收状态标记	
u8 cRxBuffer[200];
UART_HandleTypeDef UART3_Handler; //UART句柄

//初始化IO 串口1 
//bound:波特率
void uart3_init(u32 bound)
{	
	//UART 初始化设置
	UART3_Handler.Instance=USART3;					    //USART3
	UART3_Handler.Init.BaudRate=bound;				    //波特率
	UART3_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART3_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART3_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART3_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART3_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART3_Handler);					    //HAL_UART_Init()会使能UART3
	
	 /* 修改接收缓冲区大小 */
  UART3_Handler.pRxBuffPtr = cRxBuffer;
  UART3_Handler.RxXferSize = sizeof(cRxBuffer);
  UART3_Handler.RxXferCount = sizeof(cRxBuffer);
}
#endif

#if EN_UART4_RX
u8 dRxBuffer[RXBUFFERSIZE];
UART_HandleTypeDef UART4_Handler; //UART句柄

//初始化IO 串口1 
//bound:波特率
void uart4_init(u32 bound)
{	
	//UART 初始化设置
	UART4_Handler.Instance=UART4;					    //UART4
	UART4_Handler.Init.BaudRate=bound;				    //波特率
	UART4_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART4_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART4_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART4_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART4_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART4_Handler);					    //HAL_UART_Init()会使能UART4
	
	HAL_UART_Receive_IT(&UART4_Handler, (u8 *)dRxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
}

//串口4中断服务程序
void UART4_IRQHandler(void)                	
{ 	
	u32 timeout=0;
	
	HAL_UART_IRQHandler(&UART4_Handler);	//调用HAL库中断处理公用函数
	
  while (HAL_UART_GetState(&UART4_Handler) != HAL_UART_STATE_READY)//等待就绪
	{
		timeout++;////超时处理
		if(timeout>HAL_MAX_DELAY) break;		
	}
	
	timeout=0;
	while(HAL_UART_Receive_IT(&UART4_Handler, (u8 *)dRxBuffer, RXBUFFERSIZE) != HAL_OK)//一次处理完成之后，重新开启中断并设置RxXferCount为1
	{
	 timeout++; //超时处理
	 if(timeout>HAL_MAX_DELAY) break;	
	}
	
	__HAL_UART_CLEAR_FLAG(&UART4_Handler,UART_FLAG_RXNE);
	__HAL_UART_CLEAR_FLAG(&UART4_Handler,UART_FLAG_ORE);
} 

#endif

#if EN_UART5_RX   //如果使能了串口2接收
u8 eRxBuffer[RXBUFFERSIZE];
UART_HandleTypeDef UART5_Handler; //UART句柄

//初始化IO 串口5 
//bound:波特率
void uart5_init(u32 bound)
{	
	//UART 初始化设置
	UART5_Handler.Instance=UART5;					    //USART5
	UART5_Handler.Init.BaudRate=bound;				    //波特率
	UART5_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART5_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART5_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART5_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART5_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART5_Handler);					    //HAL_UART_Init()会使能UART2
	
	HAL_UART_Receive_IT(&UART5_Handler, (u8 *)eRxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
}

//串口5中断服务程序
void UART5_IRQHandler(void)                	
{ 	
	u32 timeout=0;
	
	HAL_UART_IRQHandler(&UART5_Handler);	//调用HAL库中断处理公用函数
	
  while (HAL_UART_GetState(&UART5_Handler) != HAL_UART_STATE_READY)//等待就绪
	{
		timeout++;////超时处理
		if(timeout>HAL_MAX_DELAY) break;		
	}
	
	timeout=0;
	while(HAL_UART_Receive_IT(&UART5_Handler, (u8 *)eRxBuffer, RXBUFFERSIZE) != HAL_OK)//一次处理完成之后，重新开启中断并设置RxXferCount为1
	{
	 timeout++; //超时处理
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




