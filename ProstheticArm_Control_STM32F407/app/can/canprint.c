#include "canprint.h"
#include "can.h"
#include "EPOS.h"

uint8_t data_ret[88]={0x00};

void MotorData_CAN_Send()
{
	
	data_ret[0]=0xEE;
	data_ret[1]=0xEE;
	data_ret[87]=0xFF;
	data_ret[86]=0xFF;
	motor_W1.Get_ActualPos(&data_ret[2]);
	motor_W1.Get_ActualVel(&data_ret[6]);
	motor_W1.Get_ActualCurrent(&data_ret[10]);
	
	motor_W2.Get_ActualPos(&data_ret[14]);
	motor_W2.Get_ActualVel(&data_ret[18]);
	motor_W2.Get_ActualCurrent(&data_ret[22]);
	
	motor_QB.Get_ActualPos(&data_ret[26]);
	motor_QB.Get_ActualVel(&data_ret[30]);
	motor_QB.Get_ActualCurrent(&data_ret[34]);
	
	motor_ZB.Get_ActualPos(&data_ret[38]);
	motor_ZB.Get_ActualVel(&data_ret[42]);
	motor_ZB.Get_ActualCurrent(&data_ret[46]);
	
	Message m[11];
	int i,j,count=0;
	for(i=0;i<11;i++)
	{
		m[i].COB_ID=0x035;
		m[i].len=8;
		m[i].RTR=0;
		for(j=0;j<8;j++)
			m[i].Data[j]=data_ret[count++];
		CAN_SendMsg(&m[i]);
	}
}