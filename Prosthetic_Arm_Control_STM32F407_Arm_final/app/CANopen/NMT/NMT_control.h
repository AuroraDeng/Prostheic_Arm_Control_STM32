#ifndef __NMT_CONTROL_H
#define __NMT_CONTROL_H

#include "can.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define REQUEST 1
#define NOT_A_REQUEST 0

/* NMT Command Specifier, sent by master to change a slave state */
/* ------------------------------------------------------------- */
/* Should not be modified */
enum NMT_Command_Specifier
{
NMT_Start_Node=0x01,
NMT_Stop_Node=0x02,
NMT_Enter_PreOperational=0x80,
NMT_Reset_Node=0x81,
NMT_Reset_Comunication=0x82,
};

void NMT_State_Change(uint16_t NodeID,uint8_t cs);
void Sync_Send();

#ifdef __cplusplus
}
#endif
#endif
