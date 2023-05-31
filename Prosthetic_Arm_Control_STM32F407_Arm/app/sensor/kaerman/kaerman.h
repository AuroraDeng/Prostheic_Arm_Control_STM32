#ifndef __KAERMAN_H
#define __KAERMAN_H

#ifdef __cplusplus
extern "C" {
#endif
	
#define Pi 	3.1415926

typedef struct 
{
		float hN_roll = 0; 
		float hN_Pitch = 0; 
		float hN_Yaw = 0; 
}IMU;
void WristPoseEstimate(float WristPos[]);
void CopeMPData(unsigned char ucData[]);
void CopeSPData(unsigned char ucData[]);
void RPY(float rpy[],const IMU& SP,const IMU& MP);
extern IMU MPlatform;
extern IMU SPlatform;

#ifdef __cplusplus
}
#endif
#endif
