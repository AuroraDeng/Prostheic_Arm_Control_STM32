#include "BWT61CL.h"
#include "string.h"
#include "usart_print.h"

struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;

//CopeWristPosDataΪ����2�жϵ��ú���������ÿ�յ�һ�����ݣ�����һ�����������
void CopeWristPosData(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//���ݲ���11�����򷵻�
	else
	{
		switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{
			//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
			case 0x51:memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:memcpy(&stcAngle,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;//��ջ�����
	}
}

void BWT61CL_Init()
{
	printf("Perform X-axis angle and Y-axis angle clearance.\r\n");
	SendCmdtoBWT(ACCCMD);//�ȴ�ģ���ڲ��Զ�У׼�ã�ģ���ڲ����Զ�������Ҫһ����ʱ��
	printf("The X-axis angle and Y-axis angle are cleared to zero.\r\n");			
	delay_ms(100);
	printf("Perform Z-axis angle clearance.\r\n");
	SendCmdtoBWT(YAWCMD);
	printf("The Z-axis angle is cleared to zero.\r\n");
}
