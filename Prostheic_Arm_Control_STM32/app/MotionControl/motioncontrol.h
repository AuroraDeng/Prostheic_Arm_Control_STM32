#ifndef __MOTIONCONTROL_H
#define __MOTIONCONTROL_H

#include "EPOS.h"
#include "usart_print.h"

#ifdef __cplusplus
 extern "C" {
#endif
static int motor_number=3; //控制电机数量

//Epos CMotor[3];
//Epos motor_W1(1,PPM);
//Epos motor_W2(2,PPM);
//Epos motor_QB(3,PPM);
//Epos motor_ZB(4,PPM);
//	 
void Wrist_Extension();
void Wrist_Flextion();
void Wrist_Ulnar_Deviation();
void Wrist_Radial_Deviation();
void Motor_Reset();
void HelloWorld();
#ifdef __cplusplus
}
#endif
#endif