#include "main.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart_get.h"
#include "motioncontrol.h"
#include "motor_test.h"
#include "usart_host_communication.h"

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

//����˶�ָ��
#define MOTOR_TASK_PRIO			2
#define MOTOR_STK_SIZE			512
void motor_task(void * pvParameters);
TaskHandle_t MotorTask_Handler;		//������

////���2�˶�ָ��
//#define MOTOR2_TASK_PRIO			2
//#define MOTOR2_STK_SIZE			512
//void motor2_task(void * pvParameters);
//TaskHandle_t Motor2Task_Handler;		//������

////���3�˶�ָ��
//#define MOTOR3_TASK_PRIO			2
//#define MOTOR3_STK_SIZE			512
//void motor3_task(void * pvParameters);
//TaskHandle_t Motor3Task_Handler;		//������

//��������
#define QUEUE_LENGTH        3
#define ITEM_SIZE       		sizeof(int32_t)
QueueHandle_t CommandQueue=xQueueCreate(QUEUE_LENGTH,ITEM_SIZE);

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
				
	//����Task2:���Ƶ��1���˶�
	xTaskCreate((TaskFunction_t	) motor_task,
							(char*			) "motor_task",
							(uint16_t		) MOTOR_STK_SIZE,
							(void * 		) NULL,
							(UBaseType_t	) MOTOR_TASK_PRIO,
							(TaskHandle_t*	) &MotorTask_Handler);
							
//  //����Task3:���Ƶ��2���˶�					
//	xTaskCreate((TaskFunction_t	) motor2_task,
//							(char*			) "motor2_task",
//							(uint16_t		) MOTOR2_STK_SIZE,
//							(void * 		) NULL,
//							(UBaseType_t	) MOTOR2_TASK_PRIO,
//							(TaskHandle_t*	) &Motor2Task_Handler);
//							
//	//����Task4:���Ƶ��3���˶�					
//	xTaskCreate((TaskFunction_t	) motor3_task,
//							(char*			) "motor3_task",
//							(uint16_t		) MOTOR3_STK_SIZE,
//							(void * 		) NULL,
//							(UBaseType_t	) MOTOR3_TASK_PRIO,
//							(TaskHandle_t*	) &Motor3Task_Handler);
							
	vTaskDelete(StartTask_Handler); //ɾ����ʼ����
                
  taskEXIT_CRITICAL();            //�˳��ٽ���		
}

void task1_task(void * pvParameters)
{
	u16 times=0;
	BaseType_t SendStatus;
	u8 i;
	
  for(;;)
  {				
		if(USART1_RX_STA&0x8000)//�жϽ����Ƿ���ɣ�0x8000=1000 0000 0000 0000/uint16_t USART1_RX_STA��bit15��������ɱ�־����1
		{	
				int32_t SendCommand[3]={0};
				Get_USART_Command(&UART1_Handler,SendCommand);	
				for(i=0;i<3;i++)
				{
					SendStatus=xQueueSend(CommandQueue,&SendCommand[i],0);
					if(SendStatus==pdPASS)
					{
						println_str(&UART1_Handler,"Send to the queue successfully!");
					}
					else
						println_str(&UART1_Handler,"Could not send to the queue!");
				}
//				vTaskPrioritySet(MotorTask_Handler,4);
		}
		else
		{
			times++;
			if(times%200==0)
				println_str(&UART1_Handler,"Please Enter the Prosthetic Arm Movement Command");
		} 
		vTaskDelay(5);  
  }
}

void motor_task(void * pvParameters)
{
	int32_t ReceiveCommand[3]={0};
	BaseType_t ReceiveStatus[3];
	u8 i;
	
	for(;;)
	{
		int32_t ReceiveCommand[3]={0};
		taskENTER_CRITICAL();	
		
		for(i=0;i<3;i++)
			ReceiveStatus[i]=xQueueReceive(CommandQueue,&ReceiveCommand[i],0);
//		vTaskPrioritySet(NULL,2);
		if(ReceiveStatus[0]==pdPASS&&ReceiveStatus[1]==pdPASS&&ReceiveStatus[2]==pdPASS)
		{
			println_str(&UART1_Handler,"Receive from the queue successfully!");
		
			Motor_W1(ReceiveCommand[0]);
			Motor_W2(ReceiveCommand[1]);
			Motor_QB(ReceiveCommand[2]);
		}
		
		taskEXIT_CRITICAL();	
//	else
//		println_str(&UART1_Handler,"Could not receive from the queue!");
	}
}

//void motor2_task(void * pvParameters)
//{
//	int32_t ReceiveCommand2=0;
//	BaseType_t xStatus;
//	
//	for(;;)
//	{
//		xStatus=xQueueReceive(CommandQueue,&ReceiveCommand2,xTicksToWait);
//		if(xStatus==pdPASS)
//		{		
//			taskENTER_CRITICAL();
//			println_str(&UART1_Handler,"Receive from the queue successfully!");
//			Motor_W2(ReceiveCommand2);
////			vTaskPrioritySet(NULL,1);
//			taskEXIT_CRITICAL();
//		}
////		else
////			println_str(&UART1_Handler,"Could not receive from the queue!");
//		ReceiveCommand2=0;
//	}
//}

//void motor3_task(void * pvParameters)
//{
//	int32_t ReceiveCommand3=0;
//	BaseType_t xStatus;
//	
//	for(;;)
//	{
//		xStatus=xQueueReceive(CommandQueue,&ReceiveCommand3,xTicksToWait);
//		if(xStatus==pdPASS)
//		{		
//			taskENTER_CRITICAL();
//			println_str(&UART1_Handler,"Receive from the queue successfully!");
//			Motor_QB(ReceiveCommand3);
////			vTaskPrioritySet(NULL,1);
//			taskEXIT_CRITICAL();
//		}
////		else
////			println_str(&UART1_Handler,"Could not receive from the queue!");
//		ReceiveCommand3=0;
//	}
//}

