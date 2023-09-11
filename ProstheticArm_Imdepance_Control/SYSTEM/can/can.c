/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
	CAN底层配置文件
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
    GPIO_InitStruct.Pull = GPIO_PULLUP;              //上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;         //快速
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
 * 函数名：CAN_Filter_Config
 * 描述  ：CAN的过滤器 配置
 * 调用  ：内部调用
 */
void CAN_Filter_Config()
{	
	/*config later*/
	CAN_FilterTypeDef sFilterConfig;
	
	/*在cube库中，CAN_FxR1与CAN_FxR2寄存器分别被拆成两段，CAN_FxR1寄存器的高16位对应着上面代码中的FilterIdHigh，低16位对应着FilterIdLow;
	而CAN_FxR2寄存器的高16位对应着FilterMaskIdHigh，低16位对应着FilterMaskIdLow*/
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;//过滤器工作模式设置：掩码模式or列表模式
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;//标准的CAN ID：16位的寄存器；扩展CAN ID：32位的寄存器
	sFilterConfig.FilterIdHigh=0x0000;
	sFilterConfig.FilterIdLow=0x0000;
	sFilterConfig.FilterMaskIdHigh=0x0000;
	sFilterConfig.FilterMaskIdLow=0xF000;
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation=CAN_FILTER_ENABLE;//使能过滤器
	sFilterConfig.SlaveStartFilterBank = 14;
	
	if(HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig)!=HAL_OK)
		println_str(&UART1_Handler,"The CAN filter config has not finished!");	
	
	sFilterConfig.FilterBank = 1;
	sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;//过滤器工作模式设置：掩码模式or列表模式
	sFilterConfig.FilterScale=CAN_FILTERSCALE_16BIT;//标准的CAN ID：16位的寄存器；扩展CAN ID：32位的寄存器
	sFilterConfig.FilterIdHigh=0xF0FF;
	sFilterConfig.FilterIdLow=0x1035;
	sFilterConfig.FilterMaskIdHigh=0x0000;
	sFilterConfig.FilterMaskIdLow=0x0000;
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO1;
	sFilterConfig.FilterActivation=CAN_FILTER_ENABLE;//使能过滤器
	sFilterConfig.SlaveStartFilterBank = 14;
	
	if(HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig)!=HAL_OK)
		println_str(&UART1_Handler,"The CAN filter config has not finished!");	
}

/*
 * 函数名：CAN_Config
 * 描述  ：完整配置CAN的功能
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void CAN_Config(void)
{
  MX_CAN1_Init();
  CAN_Filter_Config();
	
	/*##-3- Start the CAN peripheral ###########################################*/
	if(HAL_CAN_Start(&hcan1)!=HAL_OK)//！！！一定要打开can外设，不打开can收发就处于关闭状态
		println_str(&UART1_Handler,"The CAN peripheral has not started!");	

  /*##-4- Activate CAN RX notification 开中断#######################################*/
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
			
	if(rxmessage.RTR == CAN_RTR_REMOTE)//判断是远程帧还是数据帧
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
//	//再次开启中断
//	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
//	// 清除中断标志位
//  __HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_FF1);
}

/*
 * 函数名：CAN_SendMsg
 * 描述  ：CAN通信报文内容设置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */  
uint8_t CAN_SendMsg(Message *m)
{
		CANTxMsg TxMessage={0};
		uint32_t  CAN_TX_BOX0=0;
    
    //low = 0x0EFEFABCD;
    //high = 0x0ABCDEFAD;
  TxMessage.StdId = m->COB_ID;
	TxMessage.IDE = CAN_ID_STD;//标准信息类型
	if(m->RTR)
  		TxMessage.RTR = CAN_RTR_REMOTE;//远程帧
	else
  		TxMessage.RTR = CAN_RTR_DATA;//消息帧
	TxMessage.DLC = m->len;//数据帧长度
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
		return 0;//接收数据
	
	m->len = RxMessage.DLC;
			
	if(RxMessage.RTR == CAN_RTR_REMOTE)//判断是远程帧还是数据帧
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
    
  g_canx_txheader.StdId = id;         /* 标准标识符 */
  g_canx_txheader.ExtId = id;         /* 扩展标识符(29位) 标准标识符情况下，该成员无效*/
  g_canx_txheader.IDE = CAN_ID_STD;   /* 使用标准标识符 */
  g_canx_txheader.RTR = CAN_RTR_DATA; /* 数据帧 */
  g_canx_txheader.DLC = len;

  if (HAL_CAN_AddTxMessage(&hcan1, &g_canx_txheader, msg, &TxMailbox) != HAL_OK) /* 发送消息 */
  {
    return 1;
  }
  
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3)
	{}
		/* 等待发送完成,所有邮箱(有三个邮箱)为空 */
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
