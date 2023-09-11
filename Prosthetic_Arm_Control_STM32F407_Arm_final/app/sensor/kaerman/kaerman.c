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
		}
		
		if(HAL_UART_Receive(&UART4_Handler, (u8 *)dRxBuffer,IMUFrameLength,20)==HAL_OK)
		{
			CopeSPData((unsigned char*)dRxBuffer);
		}
}

#if defined (Euler_angle) && (Euler_angle)
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
		ucRxBuffer[ucRxCnt]=ucData[j];	//���յ������ݴ��뻺������
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
	ucRxCnt=0;//��ջ�����
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
			RxBuffer[RxCnt]=ucData[j];	//���յ������ݴ��뻺������
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
		RxCnt=0;//��ջ�����
}

void RPY(float rpy[],const IMU& SP,const IMU& MP)
{
	int i,j,k;
	float SP_X=SP.hN_Yaw*Pi/180.0;
	float SP_Y=SP.hN_Pitch*Pi/180.0;
	float SP_Z=SP.hN_roll*Pi/180.0;
	
	float MP_X=MP.hN_Yaw*Pi/180.0;
	float MP_Y=MP.hN_Pitch*Pi/180.0;
//	float MP_Z=MP.hN_roll*Pi/180.0;
	float MP_Z=SP_Z;
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
//	rpy[1]=-asin(R_ab[2][0]);//����ھ�ƽ̨��y���ת���������ƣ�
//	rpy[0]=-atan(R_ab[2][1]/R_ab[2][2]);//����ھ�ƽ̨��x���ת���������ƣ�
	rpy[1]=atan2(-R_ab[2][0],sqrt(pow(R_ab[0][0],2)+pow(R_ab[1][0],2)));//����ھ�ƽ̨��y���ת���������ƣ�
	rpy[0]=atan2(R_ab[2][1],R_ab[2][2]);//����ھ�ƽ̨��x���ת���������ƣ�	
	
//	if(abs(SP_X-MP_X)>40*Pi/180)
//		rpy[0]=SP_X-MP_X<0?SP_X-MP_X+360*Pi/180:SP_X-MP_X-360*Pi/180;//����ھ�ƽ̨��y���ת���������ƣ�
//	else
//		rpy[0]=SP_X-MP_X;
//	
//	if(abs(SP_Y-MP_Y)>40*Pi/180)
//		rpy[1]=SP_Y-MP_Y<0?SP_Y-MP_Y+360*Pi/180:SP_Y-MP_Y-360*Pi/180;
//	else
//		rpy[1]=SP_Y-MP_Y;//����ھ�ƽ̨��x���ת���������ƣ�
}

#else

void CopeMPData(unsigned char ucData[])
{		
	static unsigned char ucRxBuffer[13];
	static unsigned char ucRxCnt = 0;	
	int i=0,j,sum=0;
	while(ucData[i]!=0x55||ucData[i+1]!=0x55||ucData[i+2]!=0x02||ucData[i+3]!=0x08)
	{
		i++;
	}
	for(j=i;j<i+13;j++)
	{
		ucRxBuffer[ucRxCnt]=ucData[j];	//���յ������ݴ��뻺������
		if(ucRxCnt<12)
				sum+=ucRxBuffer[ucRxCnt];
		ucRxCnt++;
	}
		
	if((uint8_t)sum==ucRxBuffer[12])
	{
			MPlatform.q0 = (float)((int16_t)(ucRxBuffer[5]<<8) | ucRxBuffer[4]) / 32768;
		
			MPlatform.q1 = (float)((int16_t)(ucRxBuffer[7]<<8) | ucRxBuffer[6]) / 32768;
					
			MPlatform.q2 = (float)((int16_t)(ucRxBuffer[9]<<8) | ucRxBuffer[8]) / 32768;
		
			MPlatform.q3 = (float)((int16_t)(ucRxBuffer[11]<<8) | ucRxBuffer[10]) / 32768;
	}
	ucRxCnt=0;//��ջ�����
}

void CopeSPData(unsigned char ucData[])
{
		static unsigned char RxBuffer[13];
		static unsigned char RxCnt = 0;	
		int i=0,j,sum=0;
		while(ucData[i]!=0x55||ucData[i+1]!=0x55||ucData[i+2]!=0x02||ucData[i+3]!=0x08)
		{
			i++;
		}	
		for(j=i;j<i+13;j++)
		{
			RxBuffer[RxCnt]=ucData[j];	//���յ������ݴ��뻺������
			if(RxCnt<12)
				sum+=RxBuffer[RxCnt];
			RxCnt++;
		}		
		if((uint8_t)sum==RxBuffer[12])
		{
				SPlatform.q0 = (float)((int16_t)(RxBuffer[5]<<8) | RxBuffer[4]) / 32768;
					
				SPlatform.q1 = (float)((int16_t)(RxBuffer[7]<<8) | RxBuffer[6]) / 32768;
			
				SPlatform.q2 = (float)((int16_t)(RxBuffer[9]<<8) | RxBuffer[8]) / 32768;
			
				SPlatform.q3 = (float)((int16_t)(RxBuffer[11]<<8) | RxBuffer[10]) / 32768;
		}
		RxCnt=0;//��ջ�����
}

void RPY(float rpy[],const IMU& SP,const IMU& MP)
{
	int i,j,k;
	float SP_w = SP.q0;
	float SP_x = SP.q1;
	float SP_y = SP.q2;
	float SP_z = SP.q3;
	
	float MP_w = MP.q0;
	float MP_x = MP.q1;
	float MP_y = MP.q2;
	float MP_z = MP.q3;
	
	/*��ƽ̨��ת����*/
	float R_oaT[3][3]={{1 - 2*(SP_y*SP_y + SP_z*SP_z), 2 * (SP_x * SP_y - SP_w * SP_z), 2 * (SP_x * SP_z + SP_w * SP_y)},
										 {2 * (SP_x * SP_y + SP_w * SP_z), 1 - 2*(SP_x*SP_x + SP_z*SP_z), 2 * (SP_y * SP_z - SP_w * SP_x)},
										 {2 * (SP_x * SP_z - SP_w * SP_y), 2 * (SP_y * SP_z + SP_w * SP_x), 1 - 2*(SP_x*SP_x + SP_y*SP_y)}};
	
	/*��ƽ̨��ת����*/
	float R_ob[3][3]={{1 - 2*(MP_y*MP_y + MP_z*MP_z), 2 * (MP_x * MP_y - MP_w * MP_z), 2 * (MP_x * MP_z + MP_w * MP_y)},
									  {2 * (MP_x * MP_y + MP_w * MP_z), 1 - 2*(MP_x*MP_x + MP_z*MP_z), 2 * (MP_y * MP_z - MP_w * MP_x)},
									  {2 * (MP_x * MP_z - MP_w * MP_y), 2 * (MP_y * MP_z + MP_w * MP_x), 1 - 2*(MP_x*MP_x + MP_y*MP_y)}};
	
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
	
	rpy[1]=atan2(-R_ab[2][0],sqrt(pow(R_ab[0][0],2)+pow(R_ab[1][0],2)));//����ھ�ƽ̨��y���ת���������ƣ�
	rpy[0]=atan2(R_ab[2][1],R_ab[2][2]);//����ھ�ƽ̨��x���ת���������ƣ�	
}

#endif