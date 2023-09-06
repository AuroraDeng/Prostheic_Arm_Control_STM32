#include "canprint.h"
#include "can.h"
#include "EPOS.h"

uint8_t data_ret[88]={0x00};

#define JACK 1


#ifdef JACK // Lixin Hengfang LTD, Jack:
void MotorData_CAN_Send(double * actual_torque)
#else
void MotorData_CAN_Send()
#endif  // END: Lixin Hengfang 
{
	float WristPos[3]={0};
	data_ret[0]=0xEE;
	data_ret[1]=0xEE;
	data_ret[87]=0xFF;
	data_ret[86]=0xFF;

	


#ifdef JACK // Lixin Hengfang LTD, Jack:

	
#else
	
	WristPoseEstimate();
	RPY(WristPos,SPlatform,MPlatform);//解算腕关节动平台相对于静平台的姿态(输出是弧度)
	
//	motor_W1.Get_ActualPos(&data_ret[2]);
	FloatToUint8(WristPos[0],&data_ret[2]);
	motor_W1.Get_ActualVel(&data_ret[6]);
	motor_W1.Get_ActualCurrent(&data_ret[10]);
	
//	motor_W2.Get_ActualPos(&data_ret[14]);
	FloatToUint8(WristPos[1],&data_ret[14]);
	motor_W2.Get_ActualVel(&data_ret[18]);
	motor_W2.Get_ActualCurrent(&data_ret[22]);
	
		
	motor_QB.Get_ActualPos(&data_ret[26]);
	motor_QB.Get_ActualVel(&data_ret[30]);
	motor_QB.Get_ActualCurrent(&data_ret[34]);
	
#endif  // END: Lixin Hengfang 
	
	
	
	
	motor_ZB.Get_ActualPos(&data_ret[38]);
	motor_ZB.Get_ActualVel(&data_ret[42]);
	motor_ZB.Get_ActualCurrent(&data_ret[46]);
	
	
	*actual_torque = motor_ZB.Get_AcutalTor();
	printf("motor_ZB.Get_AcutalTor:[%5.5f]\r\n", motor_ZB.Get_AcutalTor());
	
#ifdef JACK // Lixin Hengfang LTD, Jack:
	//printf("I am here");
	//motor_ZB.MoveToPosition(1, int32_t target_position)
	printf("motor_ZB.Get_AcutalTor:[%5.5f]\r\n", motor_ZB.Get_AcutalTor());
	printf("motor_ZB.Get_ActualPos:[%d,%d,%d,%d]\r\n",data_ret[38],data_ret[39],data_ret[40],data_ret[41]);
	printf("motor_ZB.Get_ActualVel:[%d,%d,%d,%d]\r\n",data_ret[42],data_ret[43],data_ret[44],data_ret[45]);
	printf("motor_ZB.Get_ActualCurrent:[%d,%d,%d,%d]\r\n",data_ret[46],data_ret[47],data_ret[48],data_ret[49]);


		
	
#else

#endif  // END: Lixin Hengfang 


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

void FloatToUint8(float fdata,uint8_t udata[])
{
	float *p_data = &fdata;
	uint8_t *p_result = (uint8_t *)p_data;
	for (int i = 0; i < 4; i++) 
		udata[i] = *(p_result + i);
}

void FloatToUint8_V2(float fdata,uint8_t udata[])
{
		FloatBytes converter;
    converter.floatValue = fdata;

		for (int i = 0; i < 4; i++) 
			udata[i] = converter.bytes[i];
}