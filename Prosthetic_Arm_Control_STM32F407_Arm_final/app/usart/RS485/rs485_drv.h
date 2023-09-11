#ifndef __RS485_DRV_H__
#define __RS485_DRV_H__

#include "usart.h"
#include "gpio.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* User Config */

#define RS485_USART         UART3_Handler

#define RS485_RE_GPIO_PORT  GPIOG

#define RS485_RE_GPIO_PIN   GPIO_PIN_8



/* RS485 TX/RX Control */

void RS485_TX_ENABLE();

void RS485_RX_ENABLE(); 
#ifdef __cplusplus
}
#endif
#endif