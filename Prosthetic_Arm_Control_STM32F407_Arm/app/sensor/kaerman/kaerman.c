#include "kaerman.h"
#include "math.h"
#include "stdint.h"
#include "usart.h"
IMU MPlatform;
IMU SPlatform;

void WristPoseEstimate(void)
{
		if(HAL_UART_Receive(&UART2_Handler, (u8 *)bRxBuffer,IMUFrameLength,20)==HAL_OK)
		{
			CopeMPData((unsigned char*)bRxBuffer);
//			if(MPlatform.hN_Yaw<0)
//				MPlatform.hN_Yaw+=180;
//			else if(MPlatform.hN_Yaw>0)
//				MPlatform.hN_Yaw-=180;
//			printf("MPlatform:%f  , %f  , %f\r\n",MPlatform.hN_Yaw,MPlatform.hN_Pitch,MPlatform.hN_roll);
		}
		
		if(HAL_UART_Receive(&UART4_Handler, (u8 *)dRxBuffer,IMUFrameLength,20)==HAL_OK)
		{
			CopeSPData((unsigned char*)dRxBuffer);
//			printf("SPlatform:%f  , %f  , %f\r\n",SPlatform.hN_Yaw,SPlatform.hN_Pitch,SPlatform.hN_roll);
		}
}
void CopeMPData(unsigned char ucData[])
{		
	static unsigned char ucRxBuffer[11];
	static unsigned char ucRxCnt = 0;	
	int i=0,j,sum=0;
	while(ucData[i]!=0x55||ucData[i+1]!=0x55||ucData[i+2]!=0x01||ucData[i+3]!=0x06)
	{
		i++;
	}
	for(j=i;j<i+11;j++)
	{
		ucRxBuffer[ucRxCnt]=ucData[j];	//将收到的数据存入缓冲区中
		if(ucRxCnt<10)
				sum+=ucRxBuffer[ucRxCnt];
		ucRxCnt++;
	}
		
	if((uint8_t)sum==ucRxBuffer[10])
	{
			MPlatform.hN_Yaw = (float)((int16_t)(ucRxBuffer[5]<<8) | ucRxBuffer[4]) / 32768 * 180;//X
		
			MPlatform.hN_Pitch = (float)((int16_t)(ucRxBuffer[7]<<8) | ucRxBuffer[6]) / 32768 * 180;//Y
					
			MPlatform.hN_roll = (float)((int16_t)(ucRxBuffer[9]<<8) | ucRxBuffer[8]) / 32768 * 180;//Z
	}
	ucRxCnt=0;//清空缓存区
}

void CopeSPData(unsigned char ucData[])
{
		static unsigned char RxBuffer[11];
		static unsigned char RxCnt = 0;	
		int i=0,j,sum=0;
		while(ucData[i]!=0x55||ucData[i+1]!=0x55||ucData[i+2]!=0x01||ucData[i+3]!=0x06)
		{
			i++;
		}	
		for(j=i;j<i+11;j++)
		{
			RxBuffer[RxCnt]=ucData[j];	//将收到的数据存入缓冲区中
			if(RxCnt<10)
				sum+=RxBuffer[RxCnt];
			RxCnt++;
		}		
		if((uint8_t)sum==RxBuffer[10])
		{
				SPlatform.hN_Yaw = (float)((int16_t)(RxBuffer[5]<<8) | RxBuffer[4]) / 32768 * 180;//X
					
				SPlatform.hN_Pitch = (float)((int16_t)(RxBuffer[7]<<8) | RxBuffer[6]) / 32768 * 180;//Y
			
				SPlatform.hN_roll = (float)((int16_t)(RxBuffer[9]<<8) | RxBuffer[8]) / 32768 * 180;//Z
		}
		RxCnt=0;//清空缓存区
}

void RPY(float rpy[],const IMU& SP,const IMU& MP)
{
	int i,j,k;
	float SP_X=SP.hN_Yaw*Pi/180.0;
	float SP_Y=SP.hN_Pitch*Pi/180.0;
	float SP_Z=SP.hN_roll*Pi/180.0;
	
	float MP_X=MP.hN_Yaw*Pi/180.0;
	float MP_Y=MP.hN_Pitch*Pi/180.0;
	float MP_Z=MP.hN_roll*Pi/180.0;

//	float R[3][3]={{cos(SP_Z-MP_Z),-sin(SP_Z-MP_Z),0},
//								{sin(SP_Z-MP_Z),cos(SP_Z-MP_Z),0},
//								{0,0,1}};
	

	float R_oaT[3][3]={{cos(SP_Z)*cos(SP_Y),sin(SP_Z)*cos(SP_Y),-sin(SP_Y)},
										{cos(SP_Z)*sin(SP_Y)*sin(SP_X)-sin(SP_Z)*cos(SP_X),sin(SP_Z)*sin(SP_Y)*sin(SP_X)+cos(SP_Z)*cos(SP_X),cos(SP_Y)*sin(SP_X)},
										{cos(SP_Z)*sin(SP_Y)*cos(SP_X)+sin(SP_Z)*sin(SP_X),sin(SP_Z)*sin(SP_Y)*cos(SP_X)-cos(SP_Z)*sin(SP_X),cos(SP_Y)*cos(SP_X)}};
						
	float R_ob[3][3]={{cos(MP_Z)*cos(MP_Y),cos(MP_Z)*sin(MP_Y)*sin(MP_X)-sin(MP_Z)*cos(MP_X),cos(MP_Z)*sin(MP_Y)*cos(MP_X)+sin(MP_Z)*sin(MP_X)},
										{sin(MP_Z)*cos(MP_Y),sin(MP_Z)*sin(MP_Y)*sin(MP_X)+cos(MP_Z)*cos(MP_X),sin(MP_Z)*sin(MP_Y)*cos(MP_X)-cos(MP_Z)*sin(MP_X)},
										{-sin(MP_Y),cos(MP_Y)*sin(MP_X),cos(MP_Y)*cos(MP_X)}};
	
//	float R_o1b[3][3];
//	for (i = 0; i < 3; i++) 
//	{
//      for (j = 0; j < 3; j++) 
//			{
//					R_o1b[i][j] = 0;
//          for (k = 0; k < 3; k++) 
//              R_o1b[i][j] += R_o2b[i][k] * R[k][j];
//      }
//  }
	
	float R_ab[3][3];
	for (i = 0; i < 3; i++) 
	{
      for (j = 0; j < 3; j++) 
			{
					R_ab[i][j] = 0;
          for (k = 0; k < 3; k++) 
              R_ab[i][j] += R_oaT[i][k] * R_ob[k][j];
      }
  }
//	rpy[1]=-asin(R_ab[2][0]);//相对于静平台的y轴的转动（弧度制）
//	rpy[0]=-atan(R_ab[2][1]/R_ab[2][2]);//相对于静平台的x轴的转动（弧度制）
	rpy[1]=atan2(-R_ab[2][0],sqrt(pow(R_ab[0][0],2)+pow(R_ab[1][0],2)));//相对于静平台的y轴的转动（弧度制）
	rpy[0]=atan2(R_ab[2][1],R_ab[2][2]);//相对于静平台的x轴的转动（弧度制）	
	
//	MP_X=atan2(R_o1b[2][1],R_o1b[2][2]);//相对于静平台的x轴的转动（弧度制）	
//	MP_Y=atan2(-R_o1b[2][0],sqrt(pow(R_o1b[0][0],2)+pow(R_o1b[1][0],2)));//相对于静平台的y轴的转动（弧度制）
//	rpy[0]=MP_X-SP_X;//相对于静平台的y轴的转动（弧度制）
//	rpy[1]=MP_Y-SP_Y;//相对于静平台的x轴的转动（弧度制）
}