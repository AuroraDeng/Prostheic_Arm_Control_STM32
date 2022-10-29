#ifndef __FUZZYPID_H
#define __FUZZYPID_H

#ifdef __cplusplus
extern "C" {
#endif
	
#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

typedef struct 
{
	float kp,ki,kd;
	float error,lastError;//���ϴ����
	float integral,maxIntegral;//���֡������޷�
	float output,maxOutput,minOutput;//���������޷�
}PID;

void Fuzzy_PID_Regulation(PID *pid,float reference,float feedback);
void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut,float minOut);
void PID_Clear(PID *pid);	
	
#ifdef __cplusplus
}
#endif

#endif