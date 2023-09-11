#include "KWR46C.h"
#include "usart_print.h"

float Force[6];

void CopeElbowFTData(unsigned char ucData[],int size)
{
		int i=0;
		unsigned char data[28];
		union xxx { char m[4]; float n; }z;//����������:ʹ�ù�������������������ת��
		
		while(((i+27)<size)&&(ucData[i]!=0x48)&&(ucData[i+1]!=0xAA)&&(ucData[i+26]!=0x0D)&&(ucData[i+27]!=0x0A))
			i++;
		while((i+27)<size&&(ucData[i]==0x48)&&(ucData[i+1]==0xAA)&&(ucData[i+26]==0x0D)&&(ucData[i+27]==0x0A))
		{
			for(int j=0;j<28;j++,i++)
				data[j]=ucData[i];
			
			for (int j = 0; j < 4; j++)
			{
				z.m[j] = data[j + 2];
			}
			Force[0] = z.n*Gravity;
			for (int j = 0; j < 4; j++)
			{
				z.m[j] = data[j + 6];
			}
			Force[1] = z.n*Gravity;
			for (int j = 0; j < 4; j++)
			{
				z.m[j] = data[j + 10];
			}
			Force[2] = z.n*Gravity;
			for (int j = 0; j < 4; j++)
			{
				z.m[j] = data[j + 14];
			}
			Force[3] = z.n*Gravity;
			for (int j = 0; j < 4; j++)
			{
				z.m[j] = data[j + 18];
			}
			Force[4] = z.n*Gravity;
			for (int j = 0; j < 4; j++)
			{
				z.m[j] = data[j + 22];
			}
			Force[5] = z.n*Gravity;
			printf("-----------------------------------\r\n");
			printf("Fx= %2f N,Fy= %2f N,Fz= %2f N,Mx= %2f Nm,My= %2f Nm,Mz= %2f Nm\n", Force[0], Force[1], Force[2], Force[3], Force[4], Force[5]);
		}
}

//void CopeElbowFTData(unsigned char ucData[],int size)
//{
//		std::deque<unsigned char> receiveddata;//deque��C++��׼���е�һ��˫�˶���������������vector�������ڶ��е����˽��в����ɾ��Ԫ�صĲ���
//		unsigned char data[27];
//		union xxx { char m[4]; float n; }z;//����������:ʹ�ù�������������������ת��
//		
//		int ReceivedDataLangth,i;
//		for (int i = 0; i < size; i++)
//				receiveddata.push_back(ucData[i]);
//		
//		while (receiveddata.size() > 0)
//		{
//			ReceivedDataLangth = receiveddata.size();  //����Ŀǰ�յ����ݵĳ��ȣ�����ѭ����������������д������Ӱ�����
////			if ((receiveddata.at(0) == 0x48) && (receiveddata.at(1) == 0xAA)&&(receiveddata.at(26) == 0x0D) && (receiveddata.at(27) == 0x0A))
//			if ((ReceivedDataLangth >= 28) && (receiveddata.at(26) == 0x0d) && (receiveddata.at(27) == 0x0a))
//			{	
//				for (i = 0; i < 28; i++)
//				{
//					/*�� receiveddata �����е����ݰ�˳��������� data ��*/
//					data[i] = receiveddata.front();//receiveddata.front()��receiveddata�����еĵ�һ��Ԫ��ȡ����
//					receiveddata.pop_front();//receiveddata.pop_front()ɾ��receiveddata�����еĵ�һ��Ԫ�أ��Ա���һ�δ���
//				}
//				for (i = 0; i < 4; i++)
//				{
//					z.m[i] = data[i + 2];
//				}
//				Force[0] = z.n;
//				for (i = 0; i < 4; i++)
//				{
//					z.m[i] = data[i + 6];
//				}
//				Force[1] = z.n;
//				for (i = 0; i < 4; i++)
//				{
//					z.m[i] = data[i + 10];
//				}
//				Force[2] = z.n;
//				for (i = 0; i < 4; i++)
//				{
//					z.m[i] = data[i + 14];
//				}
//				Force[3] = z.n;
//				for (i = 0; i < 4; i++)
//				{
//					z.m[i] = data[i + 18];
//				}
//				Force[4] = z.n;
//				for (i = 0; i < 4; i++)
//				{
//					z.m[i] = data[i + 22];
//				}
//				Force[5] = z.n;
//				printf("-----------------------------------\r\n");
//				printf("Fx= %2f Kg,Fy= %2f Kg,Fz= %2f Kg,Mx= %2f Kg/M,My= %2f Kg/M,Mz= %2f Kg/M\n", Force[0], Force[1], Force[2], Force[3], Force[4], Force[5]);
//			}
//			else if (ReceivedDataLangth >= 28)
//			{
//				if (receiveddata.at(0) == 0x0a)
//					receiveddata.pop_front();/*�жϽ��յ��������Ƿ���0x0a�����з�����ͷ������������Ӷ������Ƴ���
//											����������ܻᷢ����ǰһ�����ݰ��Ľ�β����������һ�����ݰ������һ���ֽڱ�����Ϊ�ǵ�ǰ���ݰ��ĵ�һ���ֽڡ�
//											��ˣ������Ƴ�����ȷ�����ݰ��������ԡ�*/
//				else
//				{
//					i = 0;
//					while ((i <= ReceivedDataLangth-2) && (receiveddata.at(0) != 0x0d) && (receiveddata.at(1) != 0x0a))
//					{
//						receiveddata.pop_front();
//						i++;
//					}
//					if (receiveddata.size() >= 2)
//					{
//						receiveddata.pop_front();
//						receiveddata.pop_front();
//					}
//				}
//			}
//			else if (ReceivedDataLangth < 28)
//				break;
//		}
//}

//void CopeElbowFTData(unsigned char ucData)
//{
//	static unsigned char ucRxBuffer[150];
//	static unsigned char ucRxCnt = 0;	
//	std::deque<unsigned char> receiveddata;//deque��C++��׼���е�һ��˫�˶���������������vector�������ڶ��е����˽��в����ɾ��Ԫ�صĲ���
//	unsigned char data[27];
//	union xxx { char m[4]; float n; }z;//����������:ʹ�ù�������������������ת��
//	int i;
//	
//	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
//	if (ucRxBuffer[0]!=0x48) //����ͷ���ԣ������¿�ʼѰ��0x48AA����ͷ
//	{
//		ucRxCnt=0;
//		return;
//	}
////	if(ucRxBuffer[1]!=0xAA)
////	{
////		ucRxCnt=0;
////		return;
////	}
////	
//	if (ucRxCnt<28) //һ�ζ�ȡ28λ����
//		return;
//	else
//	{
//		for (i = 0; i < 28; i++)
//			receiveddata.push_back(ucRxBuffer[i]);

//		if ((receiveddata.at(26) == 0x0d) && (receiveddata.at(27) == 0x0a))
//		{	/*������յ������ݳ��ȴ��ڵ���28���ֽڣ�ReceivedDataLangth >= 28��,
//				�ҽ��յ������ݵĵ�27���ֽ���0x0d���س�����CR����receiveddata.at(26) == 0x0d��,
//				�ҽ��յ������ݵĵ�28���ֽ���0x0a�����з���LF����receiveddata.at(27) == 0x0a��,
//				��ô�������Ľ����Ϊ�档*/
//				for (i = 0; i < 28; i++)
//				{
//					/*�� receiveddata �����е����ݰ�˳��������� data ��*/
//					data[i] = receiveddata.front();//receiveddata.front()��receiveddata�����еĵ�һ��Ԫ��ȡ����
//					receiveddata.pop_front();//receiveddata.pop_front()ɾ��receiveddata�����еĵ�һ��Ԫ�أ��Ա���һ�δ���
//				}
//				for (i = 0; i < 4; i++)
//				{
//					z.m[i] = data[i + 2];
//				}
//				Force[0] = z.n;
//				for (i = 0; i < 4; i++)
//				{
//					z.m[i] = data[i + 6];
//				}
//				Force[1] = z.n;
//				for (i = 0; i < 4; i++)
//				{
//					z.m[i] = data[i + 10];
//				}
//				Force[2] = z.n;
//				for (i = 0; i < 4; i++)
//				{
//					z.m[i] = data[i + 14];
//				}
//				Force[3] = z.n;
//				for (i = 0; i < 4; i++)
//				{
//					z.m[i] = data[i + 18];
//				}
//				Force[4] = z.n;
//				for (i = 0; i < 4; i++)
//				{
//					z.m[i] = data[i + 22];
//				}
//				Force[5] = z.n;
//				//printf("Fx= %2f Kg,Fy= %2f Kg,Fz= %2f Kg,Mx= %2f Kg/M,My= %2f Kg/M,Mz= %2f Kg/M\n", Force[0], Force[1], Force[2], Force[3], Force[4], Force[5]);
//		}
//		ucRxCnt=0;//��ջ�����
//	}
//}


void KWR46C_Connect()
{
	RS485_TX_ENABLE();
	SendCmdtoKWR(senddata);
	SendCmdtoKWR(SetZero);
	SendCmdtoKWR(senddata);
	printf("KWR46C is connecting... .\r\n");
}

void ReadKWR46CData_Init()
{
//	  /* ʹ�ܴ��ڽ����ж� */
//    HAL_UART_Receive_IT(&UART3_Handler, (u8 *)cRxBuffer,wCount);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
		
    RS485_RX_ENABLE();
}