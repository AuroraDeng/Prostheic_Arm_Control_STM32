#ifndef _PDO_H
#define _PDO_H

#include "can.h"
#include "delay.h"

#ifdef __cplusplus
 extern "C" {
#endif
	 
//uint16_t TxPDO[4]={0x180,0x280,0x380,0x480};
//uint16_t RxPDO[4]={0x200,0x300,0x400,0x500};

uint32_t TransmitPDO(uint8_t NodeID,uint8_t TxPDO_n);//STM32接收PDO，从节点发送PDO
uint8_t ReceivePDO(uint8_t NodeID,uint8_t RxPDO_n,uint32_t data);//STM32发送PDO，从节点接收PDO

#ifdef __cplusplus
}
#endif
#endif
