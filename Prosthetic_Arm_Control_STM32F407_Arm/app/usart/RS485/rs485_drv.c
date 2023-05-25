#include "rs485_drv.h"

void RS485_TX_ENABLE(void)
{
		delay_ms(1);

		HAL_GPIO_WritePin(RS485_RE_GPIO_PORT, RS485_RE_GPIO_PIN, GPIO_PIN_SET);

		delay_ms(1);
}

void RS485_RX_ENABLE()
{
	 delay_ms(1);

   HAL_GPIO_WritePin(RS485_RE_GPIO_PORT, RS485_RE_GPIO_PIN, GPIO_PIN_RESET);

   delay_ms(1);
}