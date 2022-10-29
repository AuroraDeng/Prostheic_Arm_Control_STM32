#include "FuzzyPID.h"

void Fuzzy_PID_Regulation(PID *pid,float reference,float feedback)
{
	//更新数据
	pid->lastError=pid->error;
	pid->error=reference-feedback;
	//模糊策略确定kd，kp，ki
	
	
	
	//计算微分
	pid->output=(pid->error-pid->lastError)*pid->kd;
	//计算比例
	pid->output+=pid->error*pid->kp;
	//计算积分
	pid->integral+=pid->error*pid->ki;
	LIMIT(pid->integral,-pid->maxIntegral,pid->maxIntegral);//积分限幅
	pid->output+=pid->integral;
	//输出限幅
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

//清空一个pid的历史数据
void PID_Clear(PID *pid)
{
	pid->error=0;
	pid->lastError=0;
	pid->integral=0;
	pid->output=0;
}
