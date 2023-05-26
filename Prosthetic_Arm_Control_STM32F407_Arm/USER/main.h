#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "can.h"
#include "gpio.h"
#include "usart_print.h"
#include "rs485_drv.h"
#include "kaerman.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
extern float Angle_X_Final;
extern float Angle_Y_Final;
extern float Angle_Z_Final;
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
	
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
