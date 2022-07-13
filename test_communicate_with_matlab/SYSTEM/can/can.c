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

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_7TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
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
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */
  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
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
	CAN_FilterTypeDef *sFilterConfig;
	sFilterConfig->FilterActivation=ENABLE;//ʹ�ܹ�����
	sFilterConfig->FilterFIFOAssignment=CAN_FILTER_FIFO0;
//	sFilterConfig->FilterBank=14;
	/*��cube���У�CAN_FxR1��CAN_FxR2�Ĵ����ֱ𱻲�����Σ�CAN_FxR1�Ĵ����ĸ�16λ��Ӧ����������е�FilterIdHigh����16λ��Ӧ��FilterIdLow;
	��CAN_FxR2�Ĵ����ĸ�16λ��Ӧ��FilterMaskIdHigh����16λ��Ӧ��FilterMaskIdLow*/
	sFilterConfig->FilterIdHigh=(uint16_t)0x0000;
	sFilterConfig->FilterIdLow=(uint16_t)0x0000;
	sFilterConfig->FilterMaskIdHigh=0x0000;
	sFilterConfig->FilterMaskIdLow=0x0000;
	sFilterConfig->FilterMode=CAN_FILTERMODE_IDMASK;//����������ģʽ���ã�����ģʽor�б�ģʽ
	sFilterConfig->FilterScale=CAN_FILTERSCALE_16BIT;//��׼��CAN ID��16λ�ļĴ�������չCAN ID��32λ�ļĴ���
	
	HAL_CAN_ConfigFilter(&hcan1,sFilterConfig);
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
	HAL_CAN_Start(&hcan1);//������һ��Ҫ��can���裬����can�շ��ʹ��ڹر�״̬

  /*##-4- Activate CAN RX notification #######################################*/
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);	
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
		println_str(&UART1_Handler,"CAN Message send successfully!");
		printCANframe(TxMessage.StdId,m->RTR,TxMessage.DLC,m->Data);
		return 1;
	}
}

Message* CAN_ReceiveMsg()
{		
    CANRxMsg RxMessage = {0};
    Message *rxm;
		
    if(HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxMessage,rxm->Data)!=HAL_OK)
			return 0;//��������
		else
		{
			rxm->len = RxMessage.DLC;
			
			if(RxMessage.RTR == CAN_RTR_REMOTE)//�ж���Զ��֡��������֡
				rxm->RTR = 1;
			else
				rxm->RTR=0;
			
			if(RxMessage.IDE == CAN_ID_STD) //��׼IDor��չID
			{
				rxm->COB_ID = RxMessage.StdId;
				println_str(&UART1_Handler,"CAN Message receive successfully!");
				printCANframe(rxm->COB_ID,rxm->RTR,rxm->len,rxm->Data);
				return rxm;
			}
			else
				return 0;
		}
}
/* USER CODE END 1 */
