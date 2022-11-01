#ifndef __MOTIONCONTROL_H
#define __MOTIONCONTROL_H

#include "EPOS.h"
#include "usart_print.h"
#include "FuzzyPID.h"
#include "math.h"

#ifdef __cplusplus
 extern "C" {
#endif
//static int motor_number=3; //控制电机数量
static float PositionError[3]={0};
static float PositionErrorPre[3]={0};
static float PositionErrorPpre[3]={0};


void Wrist_Extension();
void Wrist_Flextion();
void Wrist_Ulnar_Deviation();
void Wrist_Radial_Deviation();
void Motor_Reset();
void HelloWorld();
void WristPositionControl(float PositionError[]);
	 
void Motor_W1(int32_t ReceiveCommand);
void Motor_W2(int32_t ReceiveCommand);
void Motor_QB(int32_t ReceiveCommand);
#ifdef __cplusplus
}
#endif
#endif