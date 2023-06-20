#ifndef __STEP_H
#define __STEP_H

#define pi 3.1415926

#ifdef __cplusplus
extern "C" {
#endif
	
#include "can.h"
#include "stdint.h"
#include "math.h"
#include "sdo.h"
	
typedef unsigned   char uint8_t;


void motor_control(uint8_t motor_address, float angle);
float read_motor_position(uint8_t motor_address);

void read_all_motor_position(float *position);
	

uint8_t Inverse_kinematics(float psi_x,float psi_y,float psi_z);
	
void matrixMultiply33_33(float A[][3], float B[][3], float result[][3]) ;
void matrixMultiply33_31(float A[][3], float B[][1], float result[][1]) ;
//void matrixAdd(float A[][3], float B[][3], float result[][3]) ;


float angle_slect(float angle1,float angle2);


#ifdef __cplusplus
}
#endif

#endif