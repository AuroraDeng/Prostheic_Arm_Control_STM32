#include "NMT_control.h"


void NMT_State_Change(UNS8 Node_ID,UNS8 cs)
{
	Message m;
	m.COB_ID=0x0000;
	m.RTR=NOT_A_REQUEST;
	m.len=2;
	m.Data[0]=cs;
	m.Data[1]=Node_ID;
	
	CAN_SendMsg(&m);
}
