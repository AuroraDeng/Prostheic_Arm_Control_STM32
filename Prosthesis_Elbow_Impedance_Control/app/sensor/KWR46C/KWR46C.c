#include "KWR46C.h"
#include "usart_print.h"

float Force[6];

void CopeElbowFTData(unsigned char ucData[],int size)
{
		int i=0;
		unsigned char data[28];
		union xxx { char m[4]; float n; }z;//共用体类型:使用共用体来处理数据类型转换
		
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
//		std::deque<unsigned char> receiveddata;//deque是C++标准库中的一个双端队列容器，类似于vector，可以在队列的两端进行插入和删除元素的操作
//		unsigned char data[27];
//		union xxx { char m[4]; float n; }z;//共用体类型:使用共用体来处理数据类型转换
//		
//		int ReceivedDataLangth,i;
//		for (int i = 0; i < size; i++)
//				receiveddata.push_back(ucData[i]);
//		
//		while (receiveddata.size() > 0)
//		{
//			ReceivedDataLangth = receiveddata.size();  //缓存目前收到数据的长度，以免循环过程中有新数据写入或读出影响操作
////			if ((receiveddata.at(0) == 0x48) && (receiveddata.at(1) == 0xAA)&&(receiveddata.at(26) == 0x0D) && (receiveddata.at(27) == 0x0A))
//			if ((ReceivedDataLangth >= 28) && (receiveddata.at(26) == 0x0d) && (receiveddata.at(27) == 0x0a))
//			{	
//				for (i = 0; i < 28; i++)
//				{
//					/*将 receiveddata 容器中的数据按顺序放入数组 data 中*/
//					data[i] = receiveddata.front();//receiveddata.front()将receiveddata容器中的第一个元素取出来
//					receiveddata.pop_front();//receiveddata.pop_front()删除receiveddata容器中的第一个元素，以便下一次处理
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
//					receiveddata.pop_front();/*判断接收到的数据是否以0x0a（换行符）开头，如果是则将它从队列中移除。
//											这种情况可能会发生在前一个数据包的结尾处，导致上一个数据包的最后一个字节被误认为是当前数据包的第一个字节。
//											因此，将其移除可以确保数据包的完整性。*/
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
//	std::deque<unsigned char> receiveddata;//deque是C++标准库中的一个双端队列容器，类似于vector，可以在队列的两端进行插入和删除元素的操作
//	unsigned char data[27];
//	union xxx { char m[4]; float n; }z;//共用体类型:使用共用体来处理数据类型转换
//	int i;
//	
//	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
//	if (ucRxBuffer[0]!=0x48) //数据头不对，则重新开始寻找0x48AA数据头
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
//	if (ucRxCnt<28) //一次读取28位数据
//		return;
//	else
//	{
//		for (i = 0; i < 28; i++)
//			receiveddata.push_back(ucRxBuffer[i]);

//		if ((receiveddata.at(26) == 0x0d) && (receiveddata.at(27) == 0x0a))
//		{	/*如果接收到的数据长度大于等于28个字节（ReceivedDataLangth >= 28）,
//				且接收到的数据的第27个字节是0x0d（回车符：CR）（receiveddata.at(26) == 0x0d）,
//				且接收到的数据的第28个字节是0x0a（换行符：LF）（receiveddata.at(27) == 0x0a）,
//				那么条件语句的结果就为真。*/
//				for (i = 0; i < 28; i++)
//				{
//					/*将 receiveddata 容器中的数据按顺序放入数组 data 中*/
//					data[i] = receiveddata.front();//receiveddata.front()将receiveddata容器中的第一个元素取出来
//					receiveddata.pop_front();//receiveddata.pop_front()删除receiveddata容器中的第一个元素，以便下一次处理
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
//		ucRxCnt=0;//清空缓存区
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
//	  /* 使能串口接收中断 */
//    HAL_UART_Receive_IT(&UART3_Handler, (u8 *)cRxBuffer,wCount);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
		
    RS485_RX_ENABLE();
}