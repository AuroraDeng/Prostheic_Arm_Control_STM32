#include "BWT61CL.h"
#include "string.h"
#include "usart_print.h"

struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;

//CopeWristPosData为串口2中断调用函数，串口每收到一个数据，调用一次这个函数。
void CopeWristPosData(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
			case 0x51:memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:memcpy(&stcAngle,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;//清空缓存区
	}
}

void BWT61CL_Init()
{
	printf("Perform X-axis angle and Y-axis angle clearance.\r\n");
	SendCmdtoBWT(ACCCMD);//等待模块内部自动校准好，模块内部会自动计算需要一定的时间
	printf("The X-axis angle and Y-axis angle are cleared to zero.\r\n");			
	delay_ms(100);
	printf("Perform Z-axis angle clearance.\r\n");
	SendCmdtoBWT(YAWCMD);
	printf("The Z-axis angle is cleared to zero.\r\n");
}
