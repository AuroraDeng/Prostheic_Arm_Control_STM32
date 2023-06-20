#ifndef __CANPRINT_H
#define __CANPRINT_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "can.h"
#include "kaerman.h"
#include <stdint.h>

typedef union {
    float floatValue;
    uint8_t bytes[4];
} FloatBytes;


extern uint8_t data_ret[88];
void MotorData_CAN_Send(void);
void FloatToUint8(float fdata,uint8_t udata[]);
void FloatToUint8_V2(float fdata,uint8_t udata[]);
#ifdef __cplusplus
}
#endif

#endif