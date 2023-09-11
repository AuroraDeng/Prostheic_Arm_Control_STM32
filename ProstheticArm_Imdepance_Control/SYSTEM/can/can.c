/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
	CAN�ײ������ļ�
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"
/* USER CODE BEGIN 0 */
//#define JACK 1
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
uint8_t statuscommand[CANCommandSize]={0x00};
uint8_t CAN1_RX_FLAG=0;

CANRxMsg RxMessage;
/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
	CAN_InitTypeDef		CAN1_InitConf;
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
	hcan1.Init = CAN1_InitConf;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;	
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
	HAL_CAN_Init(&hcan1);
	
	
  /* USER CODE BEGIN CAN1_Init 2 */
  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin =CAN_RX_PIN|CAN_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;              //����
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;         //����
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
//    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
//    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
//    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */
  /* USER CODE END CAN1_MspInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/*
 * ��������CAN_Filter_Config
 * ����  ��CAN�Ĺ����� ����
 * ����  ���ڲ�����
 */
void CAN_Filter_Config()
{	
	/*config later*/
	CAN_FilterTypeDef sFilterConfig;
	
	/*��cube���У�CAN_FxR1��CAN_FxR2�Ĵ����ֱ𱻲�����Σ�CAN_FxR1�Ĵ����ĸ�16λ��Ӧ����������е�FilterIdHigh����16λ��Ӧ��FilterIdLow;
	��CAN_FxR2�Ĵ����ĸ�16λ��Ӧ��FilterMaskIdHigh����16λ��Ӧ��FilterMaskIdLow*/
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;//����������ģʽ���ã�����ģʽor�б�ģʽ
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;//��׼��CAN ID��16λ�ļĴ�������չCAN ID��32λ�ļĴ���
	sFilterConfig.FilterIdHigh=0x0000;
	sFilterConfig.FilterIdLow=0x0000;
	sFilterConfig.FilterMaskIdHigh=0x0000;
	sFilterConfig.FilterMaskIdLow=0xF000;
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation=CAN_FILTER_ENABLE;//ʹ�ܹ�����
	sFilterConfig.SlaveStartFilterBank = 14;
	
	if(HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig)!=HAL_OK)
		println_str(&UART1_Handler,"The CAN filter config has not finished!");	
	
	sFilterConfig.FilterBank = 1;
	sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;//����������ģʽ���ã�����ģʽor�б�ģʽ
	sFilterConfig.FilterScale=CAN_FILTERSCALE_16BIT;//��׼��CAN ID��16λ�ļĴ�������չCAN ID��32λ�ļĴ���
	sFilterConfig.FilterIdHigh=0xF0FF;
	sFilterConfig.FilterIdLow=0x1035;
	sFilterConfig.FilterMaskIdHigh=0x0000;
	sFilterConfig.FilterMaskIdLow=0x0000;
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO1;
	sFilterConfig.FilterActivation=CAN_FILTER_ENABLE;//ʹ�ܹ�����
	sFilterConfig.SlaveStartFilterBank = 14;
	
	if(HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig)!=HAL_OK)
		println_str(&UART1_Handler,"The CAN filter config has not finished!");	
}

/*
 * ��������CAN_Config
 * ����  ����������CAN�Ĺ���
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
void CAN_Config(void)
{
  MX_CAN1_Init();
  CAN_Filter_Config();
	
	/*##-3- Start the CAN peripheral ###########################################*/
	if(HAL_CAN_Start(&hcan1)!=HAL_OK)//������һ��Ҫ��can���裬����can�շ��ʹ��ڹر�״̬
		println_str(&UART1_Handler,"The CAN peripheral has not started!");	

  /*##-4- Activate CAN RX notification ���ж�#######################################*/
//	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)
//		println_str(&UART1_Handler,"CAN RX0 notification has not opened!");	
	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING)!=HAL_OK)
		println_str(&UART1_Handler,"CAN RX1 notification has not opened!");	
}

void CAN1_RX1_IRQHandler()
{
	CANRxMsg rxmessage={0};
	Message m={0};
	HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO1,&rxmessage,m.Data);
	
	m.len = rxmessage.DLC;
			
	if(rxmessage.RTR == CAN_RTR_REMOTE)//�ж���Զ��֡��������֡
		m.RTR = 1;
	else
		m.RTR=0;

	m.COB_ID = rxmessage.ExtId;
	
	println_str(&UART1_Handler,"CAN Message receive successfully!");
	printCANframe(m.COB_ID,m.RTR,m.len,m.Data);
	
	if(m.COB_ID==0x1035&&CAN1_RX_FLAG<64)
	{		
		int i;
		for(i=0;i<8;i++)
			statuscommand[CAN1_RX_FLAG++]=m.Data[i];			
	}
	if(CAN1_RX_FLAG==64)
	{
		if(statuscommand[0]!=0xEE||statuscommand[1]!=0xEE||statuscommand[62]!=0xFF||statuscommand[63]!=0xFF)
			CAN1_RX_FLAG=0;
	}
//	//�ٴο����ж�
//	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
//	// ����жϱ�־λ
//  __HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_FF1);
}

/*
 * ��������CAN_SendMsg
 * ����  ��CANͨ�ű�����������
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */  
uint8_t CAN_SendMsg(Message *m)
{
		CANTxMsg TxMessage={0};
		uint32_t  CAN_TX_BOX0=0;
    
    //low = 0x0EFEFABCD;
    //high = 0x0ABCDEFAD;
  TxMessage.StdId = m->COB_ID;
	TxMessage.IDE = CAN_ID_STD;//��׼��Ϣ����
	if(m->RTR)
  		TxMessage.RTR = CAN_RTR_REMOTE;//Զ��֡
	else
  		TxMessage.RTR = CAN_RTR_DATA;//��Ϣ֡
	TxMessage.DLC = m->len;//����֡����
	TxMessage.TransmitGlobalTime = DISABLE;
//	if(HAL_CAN_AddTxMessage(&hcan1, &TxMessage,m->Data,(uint32_t *)CAN_TX_MAILBOX0)!=HAL_OK)
	if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,m->Data,&CAN_TX_BOX0)!=HAL_OK)
		return 0;
	else
	{
#ifdef JACK
		//println_str(&UART1_Handler,"CAN Message send successfully!");
		//printCANframe(TxMessage.StdId,m->RTR,TxMessage.DLC,m->Data);
		return 1;
#else
		println_str(&UART1_Handler,"CAN Message send successfully!");
		printCANframe(TxMessage.StdId,m->RTR,TxMessage.DLC,m->Data);
		return 1;
#endif
	}
}
	
u8 CAN_ReceiveMsg(Message* m)
{		
	if(HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxMessage,m->Data)!=HAL_OK)
		return 0;//��������
	
	m->len = RxMessage.DLC;
			
	if(RxMessage.RTR == CAN_RTR_REMOTE)//�ж���Զ��֡��������֡
		m->RTR = 1;
	else
		m->RTR=0;

	m->COB_ID = RxMessage.StdId;
	
	println_str(&UART1_Handler,"CAN Message receive successfully!");
	printCANframe(m->COB_ID,m->RTR,m->len,m->Data);
	
	return RxMessage.DLC;
}

uint8_t can_send_msg(uint32_t id, uint8_t *msg, uint8_t len)
{
	CANTxMsg g_canx_txheader={0};
  uint32_t TxMailbox = CAN_TX_MAILBOX0;
    
  g_canx_txheader.StdId = id;         /* ��׼��ʶ�� */
  g_canx_txheader.ExtId = id;         /* ��չ��ʶ��(29λ) ��׼��ʶ������£��ó�Ա��Ч*/
  g_canx_txheader.IDE = CAN_ID_STD;   /* ʹ�ñ�׼��ʶ�� */
  g_canx_txheader.RTR = CAN_RTR_DATA; /* ����֡ */
  g_canx_txheader.DLC = len;

  if (HAL_CAN_AddTxMessage(&hcan1, &g_canx_txheader, msg, &TxMailbox) != HAL_OK) /* ������Ϣ */
  {
    return 1;
  }
  
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3)
	{}
		/* �ȴ��������,��������(����������)Ϊ�� */
      return 0;
}

void StepCANRead(uint8_t NodeID,uint8_t ret[])
{
	Message sdo={0};
	Message rxm={0};
	uint32_t data=0;
	sdo.COB_ID=NodeID;
	sdo.RTR=0;
	sdo.len=2;
	sdo.Data[0]=0x36;
	sdo.Data[1]=0x6B;
	
	uint8_t msg[2]={0x36,0x6B};
	

	if(can_send_msg(NodeID, msg, 2)==0)
	{
	//		data=(sdo.Data[7]<<24)+(sdo.Data[6]<<16)+(sdo.Data[5]<<8)+sdo.Data[4];
		while(CAN_ReceiveMsg(&rxm)==0||rxm.COB_ID!=NodeID || rxm.len!=6 )
			
		{
		delay_ms(1);
//    CAN_SendMsg(&sdo);
//		printf("wait\r\t\n");
		}
//		data=(rxm.Data[3]<<24)+(rxm.Data[2]<<16)+(rxm.Data[1]<<8)+rxm.Data[0];
		int i;
		for(i=0;i<6;i++)
			ret[i]=rxm.Data[i];
		printf("okokokookokokoko\r\t\n");
	}
}

/* USER CODE END 1 */
