/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "usart_print.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

/** 
 * @brief The CAN message structure 
 * @ingroup can
 */
typedef struct {
  uint16_t COB_ID;	/**< message's ID */
  uint8_t RTR;		/**< remote transmission request£¨Ô¶¶Ë´«ÊäÇëÇó£©. (0 if not remote request message, 1 if remote request message) */
  uint8_t len;		/**< message's length (0 to 8) */
  uint8_t Data[8]; /**< message's datas */
} Message;

typedef  CAN_TxHeaderTypeDef CANTxMsg;
typedef  CAN_RxHeaderTypeDef CANRxMsg;

#define CAN_CLK_ENABLE()         		__HAL_RCC_CAN1_CLK_ENABLE();
#define CAN_TX_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE();
#define CAN_RX_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE();
#define CAN_RX_PIN                 GPIO_PIN_11
#define CAN_TX_PIN                 GPIO_PIN_12
#define CAN_TX_GPIO_PORT           GPIOA
#define CAN_RX_GPIO_PORT           GPIOA

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

void CAN_Config(void);
void CAN_Filter_Config(void);
uint8_t CAN_SendMsg(Message *m);
Message* CAN_ReceiveMsg(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

