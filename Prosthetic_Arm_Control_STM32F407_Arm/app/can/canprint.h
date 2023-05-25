#ifndef __CANPRINT_H
#define __CANPRINT_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "can.h"
	
extern uint8_t data_ret[88];
void MotorData_CAN_Send(void);
#ifdef __cplusplus
}
#endif

#endif