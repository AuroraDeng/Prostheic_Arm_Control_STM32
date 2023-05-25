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

static char YAWCMD[3] = {0XFF,0XAA,0X52};//�Ƕȳ�ʼ����ʹZ��Ƕȹ���
static char ACCCMD[3] = {0XFF,0XAA,0X67};//���ٶȼ�У׼��ʹX,Y��Ƕȹ�0
static char SLEEPCMD[3] = {0XFF,0XAA,0X60};//���߼������ߣ�����ģʽ�͹���ģʽ
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