#ifndef _USART_H
#define _USART_H

#include "sys.h"
#include "stdio.h"	
#include "stm32f4xx_hal_uart.h"

#ifdef __cplusplus
 extern "C" {
#endif
	 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.csom
//�޸�����:2017/4/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
//********************************************************************************
//V1.0�޸�˵�� 
////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define EN_USART2_RX 			1		//ʹ�ܣ�1��/��ֹ��0������2����
#define EN_USART3_RX 			1		//ʹ�ܣ�1��/��ֹ��0������2����
#define EN_UART4_RX       1
#define wCount 150

extern u8  USART1_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u8  USART2_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u8  USART3_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
/*�û����ж����һ��16λ����״̬�Ĵ���*/
extern u16 USART1_RX_STA;         		//����״̬���	
extern u16 USART2_RX_STA;         		//����״̬���	
extern u16 USART3_RX_STA;         		//����״̬���	

extern UART_HandleTypeDef UART1_Handler; //UART���
extern UART_HandleTypeDef UART2_Handler; //UART���
extern UART_HandleTypeDef UART3_Handler; //UART���
extern UART_HandleTypeDef UART4_Handler; //UART���

#define RXBUFFERSIZE   1 //�����С
extern u8 aRxBuffer[RXBUFFERSIZE];//HAL��USART����Buffer
extern u8 bRxBuffer[RXBUFFERSIZE];
extern u8 cRxBuffer[200];
extern u8 dRxBuffer[RXBUFFERSIZE];

//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart1_init(u32 bound);
void uart2_init(u32 bound);
void uart3_init(u32 bound);
void uart4_init(u32 bound);
void Serialport_Init(u32 bound1,u32 bound2,u32 bound3,u32 bound4);

#ifdef __cplusplus
}
#endif
#endif
