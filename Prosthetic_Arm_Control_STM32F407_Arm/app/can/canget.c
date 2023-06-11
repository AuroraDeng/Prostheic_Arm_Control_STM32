#include "can.h"
#include "canget.h"

void Get_CAN_Command(int32_t SendCommand[])
{
		int i,count=2;
		for(i=0;i<14;i++)
		{
			uint8_t c1=statuscommand[count];
			count++;
			uint8_t c2=statuscommand[count];
			count++;
			uint8_t c3=statuscommand[count];
			count++;
			uint8_t c4=statuscommand[count];
			count++;
			int32_t c=(c1<<24)|((c2<<16)|((c3<<8)|c4));
			SendCommand[i]=c-90;
		}
		CAN1_RX_FLAG=0;
}
