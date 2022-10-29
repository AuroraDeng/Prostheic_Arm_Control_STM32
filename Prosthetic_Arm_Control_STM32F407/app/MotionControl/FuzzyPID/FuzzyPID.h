#ifndef __FUZZYPID_H
#define __FUZZYPID_H

#ifdef __cplusplus
extern "C" {
#endif
	
#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

typedef struct 
{
	float kp,ki,kd;
	float error,lastError;//误差、上次误差
	float integral,maxIntegral;//积分、积分限幅
	float output,maxOutput,minOutput;//输出、输出限幅
}PID;

void Fuzzy_PID_Regulation(PID *pid,float reference,float feedback);
void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut,float minOut);
void PID_Clear(PID *pid);	
	
#ifdef __cplusplus
}
#endif

#endif