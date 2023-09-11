#ifndef __KWR46C_H__
#define __KWR46C_H__

#ifdef __cplusplus
 extern "C" {
#endif
#define Gravity 9.81
extern float Force[6];
	
//static char senddata[10]={0x48,0xAA,0X0D,0X0A,0,0,0,0,0,0};
static char senddata[10]={0x48,0xAA,0X0D,0X0A};
static char stopsenddata[10]={0x43,0xAA,0X0D,0X0A};
static char SetZero[10]={0x47,0xAA,0x0D,0x0A};
void KWR46C_Connect(void);	
void ReadKWR46CData_Init(void);
void CopeElbowFTData(unsigned char ucData[],int size);	 
//void CopeElbowFTData(unsigned char ucData);	 

#ifdef __cplusplus
}
#endif
#endif