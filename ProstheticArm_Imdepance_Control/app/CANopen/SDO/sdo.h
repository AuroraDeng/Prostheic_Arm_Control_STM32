#ifndef __SDO_H
#define __SDO_H 	

#include "can.h"
#include "delay.h"

#ifdef __cplusplus
 extern "C" {
#endif
//extern CanRxMsg RxMessage;

uint8_t SDO_Write(uint8_t NodeID,uint16_t Index,uint8_t SubIndex,uint32_t param);
void SDO_Read(uint8_t NodeID,uint16_t Index,uint8_t SubIndex,uint8_t ret[]);
	 
#ifdef __cplusplus
}
#endif
#endif
