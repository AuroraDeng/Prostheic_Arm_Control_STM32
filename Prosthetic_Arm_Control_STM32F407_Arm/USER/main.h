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

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define betax   76.314 
#define betay   80.473
#define R_CL    16
#define CountsPerTurn   1000
#define Harmonic   4
#define GearRatio  100
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
	
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
