#ifndef __BWT61CL_H
#define __BWT61CL_H


#ifdef __cplusplus
 extern "C" {
#endif
	 
struct SAcc
{
	short a[3];
	short T;
};
struct SGyro
{
	short w[3];
	short T;
};
struct SAngle
{
	short Angle[3];
	short T;
};

static char YAWCMD[3] = {0XFF,0XAA,0X52};//角度初始化：使Z轴角度归零
static char ACCCMD[3] = {0XFF,0XAA,0X67};//加速度计校准：使X,Y轴角度归0
static char SLEEPCMD[3] = {0XFF,0XAA,0X60};//休眠及解休眠：待机模式和工作模式
static char UARTMODECMD[3] = {0XFF,0XAA,0X61};//
static char IICMODECMD[3] = {0XFF,0XAA,0X62};

extern struct SAcc 		stcAcc;
extern struct SGyro 	stcGyro;
extern struct SAngle 	stcAngle;

extern void CopeWristPosData(unsigned char ucData);
void BWT61CL_Init();

#ifdef __cplusplus
}
#endif
#endif