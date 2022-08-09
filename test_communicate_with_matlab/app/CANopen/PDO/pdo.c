#include "pdo.h"

uint16_t TxPDO[4]={0x180,0x280,0x380,0x480};
uint16_t RxPDO[4]={0x200,0x300,0x400,0x500};

/*STM32接收PDO，从节点发送PDO*/
uint32_t TransmitPDO(uint8_t NodeID,uint8_t TxPDO_n)
{
	Message txm={0};//STM32给epos发的远程帧
	Message rxm={0};//epos给STM32发的数据帧
	uint32_t data=0;
	txm.COB_ID=TxPDO[TxPDO_n-1]+NodeID;
	txm.RTR=1;
	switch(TxPDO_n)
	{	
		case 1:
		{
			/*Statueword*/
			txm.len=2;
			break;
		}
		case 2:
		{
			/*velocity actual value*/
			txm.len=4;
			break;
		}
		case 3:
		{
			/*position actual value*/
			txm.len=4;
			break;
		}
		case 4:
		{
			/*torque actual value*/
			txm.len=2;
			break;
		}
	}
	if(CAN_SendMsg(&txm))
	{
		delay_ms(10);
		if(CAN_ReceiveMsg(&rxm)!=0)
			data=(rxm.Data[3]<<24)+(rxm.Data[2]<<16)+(rxm.Data[1]<<8)+rxm.Data[0];
	}
		return data;
}

/*STM32发送PDO，从节点接收PDO*/
uint8_t ReceivePDO(uint8_t NodeID,uint8_t RxPDO_n,uint32_t data)
{
	Message rxm={0};
	rxm.COB_ID=RxPDO[RxPDO_n-1]+NodeID;
	rxm.RTR=0;
	switch(RxPDO_n)
	{	
		case 1:
		{
			/*Controlword*/
			rxm.len=2;
			rxm.Data[0]=data&0xFF;
			rxm.Data[1]=data>>8;
			break;
		}
		case 2:
		{
			/*modes of operation*/
			rxm.len=1;
			rxm.Data[0]=data&0xFF;
			break;
		}
		case 3:
		{
			/*target velocity*/
			rxm.len=4;
			rxm.Data[0]=data&0xFF;
			rxm.Data[1]=(data>>8)&0xFF;
			rxm.Data[2]=(data>>16)&0xFF;
			rxm.Data[3]=data>>24;
			break;
		}
		case 4:
		{
			/*target torque*/
			rxm.len=2;
			rxm.Data[0]=data&0xFF;
			rxm.Data[1]=data>>8;
			break;
		}
	}
	if(CAN_SendMsg(&rxm))
		return 1;
	else
		return 0;
}
