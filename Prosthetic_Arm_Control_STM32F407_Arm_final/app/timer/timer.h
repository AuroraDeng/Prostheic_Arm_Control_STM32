#ifndef __TIMER_H
#define __TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"	
extern volatile uint32_t g_tickcount; // SysTick¼ÆÊýÆ÷
void start_timer(void);
uint32_t stop_timer(void);
	
#ifdef __cplusplus
}
#endif
#endif