#ifndef __KAERMAN_H
#define __KAERMAN_H

#ifdef __cplusplus
extern "C" {
#endif
	
#define Pi 	3.1415926
#define Euler_angle	(1u)
	
#if defined (Euler_angle) && (Euler_angle)
typedef struct 
{
		float hN_roll = 0; 
		float hN_Pitch = 0; 
		float hN_Yaw = 0; 
}IMU;
#else
typedef struct 
{
		float q0 = 0;
		float q1 = 0;
		float q2 = 0;
		float q3 = 0;
}IMU;
#endif

void WristPoseEstimate(void);
void CopeMPData(unsigned char ucData[]);
void CopeSPData(unsigned char ucData[]);
void RPY(float rpy[],const IMU& SP,const IMU& MP);
extern IMU MPlatform;
extern IMU SPlatform;

#ifdef __cplusplus
}
#endif
#endif
