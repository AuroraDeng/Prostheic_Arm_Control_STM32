#include "main.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart_get.h"
#include "motioncontrol.h"
#include "motor_test.h"
#include "BWT61CL.h"

/*����*/
//������ʼ�������ڴ�����������������
#define START_TASK_PRIO			1
#define START_STK_SIZE			128
void start_task(void * pvParameters);
TaskHandle_t StartTask_Handler;		//������

//�����˶�ָ������
#define TASK1_TASK_PRIO			3
#define TASK1_STK_SIZE			128
void task1_task(void * pvParameters);
TaskHandle_t Task1Task_Handler;		//������	 

//�󲿵���˶�ָ��
#define WRIST_TASK_PRIO			2
#define WRIST_STK_SIZE			512
void Wrist_task(void * pvParameters);
TaskHandle_t WristTask_Handler;		//������

//��λ�ô��������
#define WristPos_TASK_PRIO			3
#define WristPos_STK_SIZE			256
void WristPos_task(void * pvParameters);
TaskHandle_t WristPosTask_Handler;		//������

//�ⲿ����˶�ָ��
#define ELBOW_TASK_PRIO			2
#define ELBOW_STK_SIZE			512
void Elbow_task(void * pvParameters);
TaskHandle_t ElbowTask_Handler;		//������

/*��Ϣ����*/
//��������1:���˶�ָ��
#define QUEUE_LENGTH        3
#define ITEM_SIZE       		sizeof(int32_t)
QueueHandle_t CommandQueue=xQueueCreate(QUEUE_LENGTH,ITEM_SIZE);

//��������2������̬����
#define WRISTPOS_QUEUE_LENGTH        3
#define WRISTPOS_ITEM_SIZE       		sizeof(float)
QueueHandle_t WristPosQueue=xQueueCreate(WRISTPOS_QUEUE_LENGTH,WRISTPOS_ITEM_SIZE);

const TickType_t xTicksToWait=pdMS_TO_TICKS(100UL);

int main(void)
{
  HAL_Init();                    	//��ʼ��HAL��    
  Stm32_Clock_Init(336,8,2,7);  	//����ʱ��,168Mhz
	delay_init(168);               	//��ʼ����ʱ����
	Serialport_Init(115200,115200);              //��ʼ��USART
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
	xTaskCreate((TaskFunction_t	) task1_task,
				      (char*			) "task1_task",
				      (uint16_t		) TASK1_STK_SIZE,
				      (void * 		) NULL,
				      (UBaseType_t	) TASK1_TASK_PRIO,
				      (TaskHandle_t*	) &Task1Task_Handler);
				
	//����Task2:�����󲿵�����˶�
	xTaskCreate((TaskFunction_t	) Wrist_task,
							(char*			) "Wrist_task",
							(uint16_t		) WRIST_STK_SIZE,
							(void * 		) NULL,
							(UBaseType_t	) WRIST_TASK_PRIO,
							(TaskHandle_t*	) &WristTask_Handler);
							
  //����Task3:�����λ��							
	xTaskCreate((TaskFunction_t	) WristPos_task,
							(char*			) "WristPos_task",
							(uint16_t		) WristPos_STK_SIZE,
							(void * 		) NULL,
							(UBaseType_t	) WristPos_TASK_PRIO,
							(TaskHandle_t*	) &WristPosTask_Handler);
//							
//	//����Task4:���Ƶ��3���˶�					
//	xTaskCreate((TaskFunction_t	) Elbow_task,
//							(char*			) "Elbow_task",
//							(uint16_t		) ELBOW_STK_SIZE,
//							(void * 		) NULL,
//							(UBaseType_t	) ELBOW_TASK_PRIO,
//							(TaskHandle_t*	) &Motor3Task_Handler);
							
	vTaskDelete(StartTask_Handler); //ɾ����ʼ����
                
  taskEXIT_CRITICAL();            //�˳��ٽ���		
}

void task1_task(void * pvParameters)
{
	u16 times=0;
	BaseType_t SendStatus[3];
	u8 i;
	
  for(;;)
  {				
		if(USART1_RX_STA&0x8000)//�жϽ����Ƿ���ɣ�0x8000=1000 0000 0000 0000/uint16_t USART1_RX_STA��bit15��������ɱ�־����1
		{	
			taskENTER_CRITICAL();	//�����ٽ�״̬
			
				int32_t SendCommand[3]={0};
				Get_USART_Command(&UART1_Handler,SendCommand);	
				
				for(i=0;i<3;i++)
					SendStatus[i]=xQueueSend(CommandQueue,&SendCommand[i],0);
				
				if(SendStatus[0]==pdPASS&&SendStatus[1]==pdPASS&&SendStatus[2]==pdPASS)
				{
					println_str(&UART1_Handler,"Send to CommandQueue successfully!");
//					vTaskPrioritySet(WristTask_Handler,4);
				}
					else
						println_str(&UART1_Handler,"Could not send to CommandQueue!");
			taskEXIT_CRITICAL();	//�˳��ٽ�״̬
		}
		else
		{
			times++;
			if(times%200==0)
			{
				taskENTER_CRITICAL();	//�����ٽ�״̬
				println_str(&UART1_Handler,"Please Enter the Prosthetic Arm Movement Command");
				taskEXIT_CRITICAL();	//�˳��ٽ�״̬
			}
		} 
		vTaskDelay(5);  
  }
}

void Wrist_task(void * pvParameters)
{
	BaseType_t CommandStatus[3];
	BaseType_t WristPosStatus[3];
	u8 i;
	
	for(;;)
	{
		int32_t WristTargetPos[3]={0};
		float WristPos[3]={0};
		
		do
		{
			/*��ȡ��֫���ʵʱ��̬��Ŀ����̬*/
			for(i=0;i<3;i++)
			{
				CommandStatus[i]=xQueueReceive(CommandQueue,&WristTargetPos[i],0);
				WristPosStatus[i]=xQueueReceive(WristPosQueue,&WristPos[i],0);
				PositionError[i]=(float)WristTargetPos[i]-WristPos[i];//��֫���ʵʱλ��ƫ��
			}
		
			if(WristPosStatus[0]==pdPASS&&WristPosStatus[1]==pdPASS&&WristPosStatus[2]==pdPASS&&CommandStatus[0]==pdPASS&&CommandStatus[1]==pdPASS&&CommandStatus[2]==pdPASS)
			{
				println_str(&UART1_Handler,"Receive from the queue successfully!");

				taskENTER_CRITICAL();	//�����ٽ�״̬
				
				WristPositionControl(PositionError);
				
				taskEXIT_CRITICAL();	//�˳��ٽ�״̬
			 }
		}while(PositionError[0]>1||PositionError[1]>1||PositionError[2]>1);
		vTaskPrioritySet(NULL,2);
	}
}

void WristPos_task(void * pvParameters)
{
	float WristPos[3];
	BaseType_t SendStatus[3];
	u8 i;
	
	void BWT61CL_Init();
	
  for(;;)
  {			
		taskENTER_CRITICAL();	//�����ٽ�״̬
		printf("-----------------------------------\r\n");
//		//������ٶ�
//		//���ڽ��ܵ��������Ѿ���������Ӧ�Ľṹ��ı������ˣ�����˵�����Э�飬�Լ��ٶ�Ϊ�� stcAcc.a[0]/32768*16����X��ļ��ٶȣ�
//		printf("Acc:%.3f %.3f %.3f\r\n",(float)stcAcc.a[0]/32768*16,(float)stcAcc.a[1]/32768*16,(float)stcAcc.a[2]/32768*16);
//		vTaskDelay(10);
//		//������ٶ�
//		printf("Gyro:%.3f %.3f %.3f\r\n",(float)stcGyro.w[0]/32768*2000,(float)stcGyro.w[1]/32768*2000,(float)stcGyro.w[2]/32768*2000);
//		vTaskDelay(10);
		for(i=0;i<3;i++)
		{
			WristPos[i]=(float)stcAngle.Angle[i]/32768*180;
			SendStatus[i]=xQueueSend(WristPosQueue,&WristPos[i],0);
		}
		
		if(SendStatus[0]==pdPASS&&SendStatus[1]==pdPASS&&SendStatus[2]==pdPASS)
		{
			println_str(&UART1_Handler,"Send to WristPosQueue successfully!");
//			vTaskPrioritySet(WristTask_Handler,3);
		}
		else
			println_str(&UART1_Handler,"Could not send to WristPosQueue!");
		
		//����Ƕ�
		printf("Angle:%.3f %.3f %.3f\r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180,(float)stcAngle.Angle[2]/32768*180);
		
		taskEXIT_CRITICAL();	//�˳��ٽ�״̬
//		vTaskDelay(1000);//�ȴ��������
		vTaskDelay(10);//�ȴ��������
  }
}

void Elbow_task(void * pvParameters)
{

}