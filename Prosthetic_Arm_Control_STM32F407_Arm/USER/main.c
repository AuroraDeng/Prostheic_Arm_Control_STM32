#include "main.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "usart_print.h"
#include "usart_get.h"
#include "canget.h"
#include "canprint.h"
#include "motioncontrol.h"
#include "math.h"
#include "step.h"
#include "KWR46C.h"
#include "HWT101.h"
#include "kaerman.h"

/*����*/
//������ʼ�������ڴ�����������������
#define START_TASK_PRIO			1
#define START_STK_SIZE			128
void start_task(void * pvParameters);
TaskHandle_t StartTask_Handler;		//������

//�����˶�ָ������
#define COMMAND_TASK_PRIO			3
#define COMMAND_STK_SIZE			512
void Command_task(void * pvParameters);
TaskHandle_t CommandTask_Handler;		//������	 

//�󲿵���˶�ָ��
#define WRIST_TASK_PRIO			3
#define WRIST_STK_SIZE			256
void Wrist_task(void * pvParameters);
TaskHandle_t WristTask_Handler;		//������

//��λ�ô��������
#define WristPos_TASK_PRIO		2
#define WristPos_STK_SIZE			512
void WristPos_task(void * pvParameters);
TaskHandle_t WristPosTask_Handler;		//������

//�ⲿ����˶�ָ��
#define ELBOW_TASK_PRIO			2
#define ELBOW_STK_SIZE			512
void Elbow_task(void * pvParameters);
TaskHandle_t ElbowTask_Handler;		//������

//�ⲿ���ؼ��˶�״̬���
#define ElbowStatus_TASK_PRIO		2
#define ElbowStatus_STK_SIZE			256
void ElbowStatus_task(void * pvParameters);
TaskHandle_t ElbowStatusTask_Handler;		//������

/*��Ϣ����*/
//��������1:���˶�ָ��
#define QUEUE_LENGTH        3
#define ITEM_SIZE       		sizeof(float)
QueueHandle_t WristCommandQueue=xQueueCreate(QUEUE_LENGTH,ITEM_SIZE);

//��������2������̬����
#define WRISTPOS_QUEUE_LENGTH        3
#define WRISTPOS_ITEM_SIZE       		sizeof(float)
QueueHandle_t WristPosQueue=xQueueCreate(WRISTPOS_QUEUE_LENGTH,WRISTPOS_ITEM_SIZE);

//��������3:�ⲿ�˶�ָ��
#define ELBOWCOMMAND_QUEUE_LENGTH        1
#define ELBOWCOMMAND_ITEM_SIZE       		sizeof(float)
QueueHandle_t ElbowCommandQueue=xQueueCreate(ELBOWCOMMAND_QUEUE_LENGTH,ELBOWCOMMAND_ITEM_SIZE);

//��������4����ؽ�״̬����
#define ElbowKdata_QUEUE_LENGTH        4
#define ElbowKdata_ITEM_SIZE       		sizeof(float)
QueueHandle_t ElbowKdataQueue=xQueueCreate(ElbowKdata_QUEUE_LENGTH,ElbowKdata_ITEM_SIZE);

/*�¼���־��*/
EventGroupHandle_t EventGroupHandler=xEventGroupCreate();

const TickType_t xTicksToWait=pdMS_TO_TICKS(100UL);

int main(void)
{
  HAL_Init();                    	//��ʼ��HAL��    
  Stm32_Clock_Init(336,8,2,7);  	//����ʱ��,168Mhz
	delay_init(168);               	//��ʼ����ʱ����
	MX_GPIO_Init();
	Serialport_Init(115200,115200,460800,115200,9600);              //��ʼ��USART
	CAN_Config();

	//������ʼ����
   xTaskCreate((TaskFunction_t )start_task,            //������
               (const char*    )"start_task",          //��������
               (uint16_t       )START_STK_SIZE,        //�����ջ��С
               (void*          )NULL,                  //���ݸ��������Ĳ���
               (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
               (TaskHandle_t*  )&StartTask_Handler);   //������  

	vTaskStartScheduler();          //�����������
}

void start_task(void * pvParameters)
{
	taskENTER_CRITICAL();           //�����ٽ���
	
	//��������1�����մ����˶�ָ��
	xTaskCreate((TaskFunction_t	) Command_task,
				      (char*			) "Command_task",
				      (uint16_t		) COMMAND_STK_SIZE,
				      (void * 		) NULL,
				      (UBaseType_t	) COMMAND_TASK_PRIO,
				      (TaskHandle_t*	) &CommandTask_Handler);
				
//	//����Task2:�����󲿵�����˶�
//	xTaskCreate((TaskFunction_t	) Wrist_task,
//							(char*			) "Wrist_task",
//							(uint16_t		) WRIST_STK_SIZE,
//							(void * 		) NULL,
//							(UBaseType_t	) WRIST_TASK_PRIO,
//							(TaskHandle_t*	) &WristTask_Handler);
//			
#if	WristPosReport						
  //����Task3:�����λ��							
	xTaskCreate((TaskFunction_t	) WristPos_task,
							(char*			) "WristPos_task",
							(uint16_t		) WristPos_STK_SIZE,
							(void * 		) NULL,
							(UBaseType_t	) WristPos_TASK_PRIO,
							(TaskHandle_t*	) &WristPosTask_Handler);
#endif
//							
//	//����Task4:���Ƶ��3���˶�					
//	xTaskCreate((TaskFunction_t	) Elbow_task,
//							(char*			) "Elbow_task",
//							(uint16_t		) ELBOW_STK_SIZE,
//							(void * 		) NULL,
//							(UBaseType_t	) ELBOW_TASK_PRIO,
//							(TaskHandle_t*	) &ElbowTask_Handler);

//  //����Task5:�����ؽ����ؼ��˶�״̬							
//	xTaskCreate((TaskFunction_t	) ElbowStatus_task,
//							(char*			) "ElbowStatus_task",
//							(uint16_t		) ElbowStatus_STK_SIZE,
//							(void * 		) NULL,
//							(UBaseType_t	) ElbowStatus_TASK_PRIO,
//							(TaskHandle_t*	) &ElbowStatusTask_Handler);
							
	vTaskDelete(StartTask_Handler); //ɾ����ʼ����
                
  taskEXIT_CRITICAL();            //�˳��ٽ���		
}

void Command_task(void * pvParameters)
{	
  for(;;)
  {	
			taskENTER_CRITICAL();	//�����ٽ�״̬
			if((USART1_RX_STA&0x8000)||(CAN1_RX_FLAG & 0x40))//�жϽ����Ƿ���ɣ�0x8000=1000 0000 0000 0000/uint16_t USART1_RX_STA��bit15��������ɱ�־����1
			{	
				int32_t SendCommand[14]={0};
				if(CAN1_RX_FLAG & 0x40)
					Get_CAN_Command(SendCommand);
				if(USART1_RX_STA&0x8000)
					Get_USART_Command(&UART1_Handler,SendCommand);	
				
				WristPostureControl(SendCommand);//��ƽ̨��̬����
				Motor_QB(SendCommand[4]);
				Motor_ZB(SendCommand[6]);
				Inverse_kinematics(SendCommand[8],SendCommand[10],SendCommand[12]);
				MotorData_CAN_Send();
			}   
			taskEXIT_CRITICAL();	//�˳��ٽ�״̬
			vTaskDelay(2);                                     
	}
}



//void Wrist_task(void * pvParameters)
//{
//	BaseType_t CommandStatus[3];
//	BaseType_t WristPosStatus[3];
//	u8 i;
//	
//	for(;;)
//	{
//		while(PositionError[0]>1||PositionError[1]>1||PositionError[2]>1)
//		{	
//			if(xEventGroupWaitBits(EventGroupHandler,0x05,pdTRUE,pdTRUE,0))
//			{
//				float WristTargetPos[3]={0};
//				float WristPos[3]={0};
//				
//				/*��ȡ��֫���ʵʱ��̬��Ŀ����̬*/
//				for(i=0;i<3;i++)
//				{
//					CommandStatus[i]=xQueueReceive(WristCommandQueue,&WristTargetPos[i],0);
//					WristPosStatus[i]=xQueueReceive(WristPosQueue,&WristPos[i],0);
//					if(WristPosStatus[i]==pdPASS&&CommandStatus[i]==pdPASS)
//					{					
//						println_str(&UART1_Handler,"Receive from the queue successfully!");
//						PositionError[i]=WristTargetPos[i]-WristPos[i];//��֫���ʵʱλ��ƫ��
//					}
//				}
//				taskENTER_CRITICAL();	//�����ٽ�״̬	
//				
//					if(WristTargetPos[0]>60||WristTargetPos[1]>60||WristTargetPos[0]<-60||WristTargetPos[1]<-60)
//					{
//							Motor_W1(0);
//							Motor_W2(0);
//					}
//					else
//					{
//							WristPositionControl(PositionError);
//							vTaskPrioritySet(NULL,2);
//					}
//				
//				taskEXIT_CRITICAL();	//�˳��ٽ�״̬
//			}
//			vTaskDelay(2);
//		}
////		vTaskDelay(1);
//	}
//}

void WristPos_task(void * pvParameters)
{
	float WristPos[2];
	BaseType_t SendStatus[3];
	u8 i;
	
  for(;;)
  {	
		taskENTER_CRITICAL();	//�����ٽ�״̬
		if(HAL_UART_Receive(&UART2_Handler, (u8 *)bRxBuffer,IMUFrameLength,20)==HAL_OK)
		{
			CopeMPData((unsigned char*)bRxBuffer);	
			printf("MPlatform: X:%f  , Y:%f  , Z:%f\r\n",MPlatform.hN_Yaw,MPlatform.hN_Pitch,MPlatform.hN_roll);
		}

		if(HAL_UART_Receive(&UART4_Handler, (u8 *)dRxBuffer,IMUFrameLength,20)==HAL_OK)
		{
			CopeSPData((unsigned char*)dRxBuffer);
			printf("SPlatform:X:%f  , Y:%f  , Z:%f\r\n",SPlatform.hN_Yaw,SPlatform.hN_Pitch,SPlatform.hN_roll);
		}
		
		RPY(WristPos,SPlatform,MPlatform);//������ؽڶ�ƽ̨����ھ�ƽ̨����̬
		printf("WristPos:%f  , %f  \r\n",WristPos[0]*180/Pi,WristPos[1]*180/Pi);
		taskEXIT_CRITICAL();	//�˳��ٽ�״̬
		
		vTaskDelay(100);//�ȴ��������
  }
}

//void Elbow_task(void * pvParameters)
//{
//	
//	BaseType_t CommandStatus;
//	for(;;)
//	{
//		taskENTER_CRITICAL();	//�����ٽ�״̬
//		if(xEventGroupWaitBits(EventGroupHandler,0x02,pdTRUE,pdTRUE,0)&0x02)
//		{
//			float ReceiveCommand=0;
//			CommandStatus=xQueueReceive(ElbowCommandQueue,&ReceiveCommand,0);
//			if(CommandStatus==pdPASS)
//				Elbow_CompliantControl(ReceiveCommand);
//			else
//				println_str(&UART1_Handler,"Could not Receive Elbow Command!");
//		}
//		taskEXIT_CRITICAL();	//�˳��ٽ�״̬
//		vTaskDelay(100);
//	}
//}

//void ElbowStatus_task(void * pvParameters)
//{
//	float EKdata[3];
//	BaseType_t SendStatus[3];
//	int i;
//	
//	KWR46C_Connect();
//	ReadKWR46CData_Init();	
//	WitInit(WIT_PROTOCOL_NORMAL, 0x50);//��ʼ���Ƕȴ�������׼Э�飬�����豸��ַ(Ϊʲô���ó�0x50)
//	WitSerialWriteRegister(SendCmdtoHWT);//ע��д�ص�����
//	WitRegisterCallBack(SensorDataUpdata);//ע���ȡ���������ݻص���������������״̬����ʾ���������Ѿ��ش���
//	WitDelayMsRegister(delay_ms);
//	AutoScanSensor();

//	for(;;)
//	{
//		taskENTER_CRITICAL();	//�����ٽ�״̬
//		if(HAL_UART_Receive(&UART3_Handler, (u8 *)cRxBuffer, wCount,20)==HAL_OK)
//			CopeElbowFTData((unsigned char *)cRxBuffer,wCount);
//		if(xQueueSend(ElbowKdataQueue,&Force[5],0)==pdPASS)
//				println_str(&UART1_Handler,"FTData send to ElbowKdataQueue successfully!");
//		else
//				println_str(&UART1_Handler,"FTData could not send to ElbowKdataQueue!");
//		CopeElbowKData();
//		for(i=0;i<3;i++)
//		{
//				float temp;
//				if(i==0)
//					temp=(float)ElbowKdata.fAngle[2];
//				else if(i==1)
//					temp=(float)ElbowKdata.fGyro[2];
//				else if(i==2)
//					temp=(float)ElbowKdata.fAcc[2];
//				EKdata[i]=temp;
//				SendStatus[i]=xQueueSend(ElbowKdataQueue,&EKdata[i],0);
//		}
//		if(SendStatus[0]==pdPASS&&SendStatus[1]==pdPASS&&SendStatus[2]==pdPASS)
//				println_str(&UART1_Handler,"KData send to ElbowKdataQueue successfully!");
//			else
//				println_str(&UART1_Handler,"KData could not send to ElbowKdataQueue!");
//		taskEXIT_CRITICAL();	//�˳��ٽ�״̬
//			
//		vTaskDelay(100);//�ȴ��������
//	}
//}