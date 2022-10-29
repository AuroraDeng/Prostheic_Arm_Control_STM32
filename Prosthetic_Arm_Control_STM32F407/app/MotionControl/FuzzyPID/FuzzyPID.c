#include "FuzzyPID.h"

void Fuzzy_PID_Regulation(PID *pid,float reference,float feedback)
{
	//��������
	pid->lastError=pid->error;
	pid->error=reference-feedback;
	//ģ������ȷ��kd��kp��ki
	
	
	
	//����΢��
	pid->output=(pid->error-pid->lastError)*pid->kd;
	//�������
	pid->output+=pid->error*pid->kp;
	//�������
	pid->integral+=pid->error*pid->ki;
	LIMIT(pid->integral,-pid->maxIntegral,pid->maxIntegral);//�����޷�
	pid->output+=pid->integral;
	//����޷�
	LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}

void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut,float minOut)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
	pid->maxIntegral=maxI;
	pid->maxOutput=maxOut;
	pid->minOutput=minOut;
}

//���һ��pid����ʷ����
void PID_Clear(PID *pid)
{
	pid->error=0;
	pid->lastError=0;
	pid->integral=0;
	pid->output=0;
}
