#include "sdo.h"

uint8_t SDO_Write(uint8_t NodeID,uint16_t Index,uint8_t SubIndex,uint32_t param)
{
	Message sdo={0};
	sdo.COB_ID=0x600+NodeID;
	sdo.RTR=0;
	sdo.len=8;
	sdo.Data[3]=SubIndex;
	sdo.Data[2]=Index>>8;
	sdo.Data[1]=Index&0xFF;
	sdo.Data[0]=0x23;
	
	sdo.Data[7]=param>>24;
	sdo.Data[6]=(param>>16)&0xFF;
	sdo.Data[5]=(param>>8)&0xFF;
	sdo.Data[4]=param&0xFF;
	
	if(CAN_SendMsg(&sdo))
		return 1;
	else
		return 0;
}

void SDO_Read(uint8_t NodeID,uint16_t Index,uint8_t SubIndex,uint8_t ret[])
{
	Message sdo={0};
	Message rxm={0};
	uint32_t data=0;
	sdo.COB_ID=0x600+NodeID;
	sdo.RTR=0;
	sdo.len=8;
	sdo.Data[3]=SubIndex;
	sdo.Data[2]=Index>>8;
	sdo.Data[1]=Index&0xFF;
	sdo.Data[0]=0x40;
	
	sdo.Data[7]=0x00;
	sdo.Data[6]=0x00;
	sdo.Data[5]=0x00;
	sdo.Data[4]=0x00;
	
	if(CAN_SendMsg(&sdo))
	{
	//		data=(sdo.Data[7]<<24)+(sdo.Data[6]<<16)+(sdo.Data[5]<<8)+sdo.Data[4];
		while(CAN_ReceiveMsg(&rxm)==0||rxm.COB_ID!=0x580+NodeID)
			delay_ms(1);
//		data=(rxm.Data[3]<<24)+(rxm.Data[2]<<16)+(rxm.Data[1]<<8)+rxm.Data[0];
		int i;
		for(i=0;i<4;i++)
			ret[i]=rxm.Data[i];
	}
}
