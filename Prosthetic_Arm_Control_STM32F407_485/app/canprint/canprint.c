#include "canprint.h"
#include "can.h"
#include "motioncontrol.h"
#include "EPOS.h"

uint8_t data_ret[88]={0x00};

void Arm_MotorData_Ret()
{
	
	data_ret[0]=0xEE;
	data_ret[0]=0xEE;
	
	data_ret[87]=0xFF;
	data_ret[86]=0xFF;
	
}