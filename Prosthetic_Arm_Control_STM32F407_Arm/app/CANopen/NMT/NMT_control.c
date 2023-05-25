#include "NMT_control.h"


void NMT_State_Change(uint16_t NodeID,uint8_t cs)
{
	Message m;
	m.COB_ID=0x0000;
	m.RTR=NOT_A_REQUEST;
	m.len=2;
	m.Data[0]=cs;
	m.Data[1]=NodeID;
	
	CAN_SendMsg(&m);
}

void Sync_Send()
{
	Message m;
	m.COB_ID=0x80;
	m.RTR=NOT_A_REQUEST;
	m.len=1;
	m.Data[0]=0;
	
	CAN_SendMsg(&m);
}
