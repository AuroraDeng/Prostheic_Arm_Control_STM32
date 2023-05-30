#include "kaerman.h"
#include "math.h"
#include "stdint.h"

IMU MPlatform;
IMU SPlatform;

void CopeMPData(unsigned char ucData[])
{		
	static unsigned char ucRxBuffer[11];
	static unsigned char ucRxCnt = 0;	
	int i,j;
	for(i=0;i<21;i++)
	{
		if (ucData[i]==0x55&&ucData[i+1]==0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
			break;
	}
	for(j=i;j<i+11;j++)
		ucRxBuffer[ucRxCnt++]=ucData[j];	//���յ������ݴ��뻺������
	if((ucRxBuffer[1]==0x55)&&(ucRxBuffer[2]==0x01))
	{
			MPlatform.hN_roll = (float)((int16_t)(ucRxBuffer[4]<<8) | ucRxBuffer[5]) / 32768 * 180;
					
			MPlatform.hN_Pitch = (float)((int16_t)(ucRxBuffer[6]<<8) | ucRxBuffer[7]) / 32768 * 180;
					
			MPlatform.hN_Yaw = (float)((int16_t)(ucRxBuffer[8]<<8) | ucRxBuffer[9]) / 32768 * 180;
	}
	ucRxCnt=0;//��ջ�����
}

void CopeSPData(unsigned char ucData[])
{
		static unsigned char RxBuffer[11];
		static unsigned char RxCnt = 0;	
		int i,j;
		for(i=0;i<21;i++)
		{
			if (ucData[i]==0x55&&ucData[i+1]==0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
				break;
		}			
		for(j=i;j<i+11;j++)
				RxBuffer[RxCnt++]=ucData[j];	//���յ������ݴ��뻺������
		if((RxBuffer[1]==0x55)&&(RxBuffer[2]==0x01))
		{
				SPlatform.hN_roll = (float)((int16_t)(RxBuffer[4]<<8) | RxBuffer[5]) / 32768 * 180;
					
				SPlatform.hN_Pitch = (float)((int16_t)(RxBuffer[6]<<8) | RxBuffer[7]) / 32768 * 180;
					
				SPlatform.hN_Yaw = (float)((int16_t)(RxBuffer[8]<<8) | RxBuffer[9]) / 32768 * 180;
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
	float MP_Z=MP.hN_roll*Pi/180.0;
	
	float R_oaT[3][3]={{cos(SP_Z)*cos(SP_Y),sin(SP_Z)*cos(SP_Y),-sin(SP_Y)},
										{cos(SP_Z)*sin(SP_Y)*sin(SP_X)-sin(SP_Z)*cos(SP_X),sin(SP_Z)*sin(SP_Y)*sin(SP_X)+cos(SP_Z)*cos(SP_X),cos(SP_Y)*sin(SP_X)},
										{cos(SP_Z)*sin(SP_Y)*cos(SP_X)+sin(SP_Z)*sin(SP_X),sin(SP_Z)*sin(SP_Y)*cos(SP_X)-cos(SP_Z)*sin(SP_X),cos(SP_Y)*cos(SP_X)}};
											
	float R_ob[3][3]={{cos(MP_Z)*cos(MP_Y),cos(MP_Z)*sin(MP_Y)*sin(MP_X)-sin(MP_Z)*cos(MP_X),cos(MP_Z)*sin(MP_Y)*cos(MP_X)+sin(MP_Z)*sin(MP_X)},
										{sin(MP_Z)*cos(MP_Y),sin(MP_Z)*sin(MP_Y)*sin(MP_X)+cos(MP_Z)*cos(MP_X),sin(MP_Z)*sin(MP_Y)*cos(MP_X)-cos(MP_Z)*sin(MP_X)},
										{-sin(MP_Y),cos(MP_Y)*sin(MP_X),cos(MP_Y)*cos(MP_X)}};
	
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
//	rpy[0]=-asin(R_ab[1][2])*180/Pi;//����ھ�ƽ̨��x���ת�����Ƕ��ƣ�
//	rpy[1]=-asin(R_ab[2][0])*180/Pi;//����ھ�ƽ̨��y���ת�����Ƕ��ƣ�
	rpy[0]=-asin(R_ab[1][2]);//����ھ�ƽ̨��x���ת���������ƣ�
	rpy[1]=-asin(R_ab[2][0]);//����ھ�ƽ̨��y���ת���������ƣ�
}