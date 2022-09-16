#include "main.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart_get.h"
#include "motioncontrol.h"
#include "motor_test.h"
#include "usart_host_communication.h"

//
#define TASK1_TASK_PRIO			2
#define TASK1_STK_SIZE			128
void task1_task(void * pvParameters);
TaskHandle_t Task1Task_Handler;		//任务句柄	 
 
#define TASK2_TASK_PRIO			1
#define TASK2_STK_SIZE			256000
void task2_task(void * pvParameters);
TaskHandle_t Task2Task_Handler;		//任务句柄

//
#define QUEUE_LENGTH        1
#define ITEM_SIZE       		sizeof(uint32_t)
QueueHandle_t CommandQueue=xQueueCreate(QUEUE_LENGTH,ITEM_SIZE);

int main(void)
{
  HAL_Init();                    	//初始化HAL库    
  Stm32_Clock_Init(336,8,2,7);  	//设置时钟,168Mhz
	delay_init(168);               	//初始化延时函数
	Serialport_Init(115200,115200);              //初始化USART
	CAN_Config();

	//创建任务1：接收串口运动指令
	xTaskCreate((TaskFunction_t	) task1_task,
				(char*			) "task1_task",
				(uint16_t		) TASK1_STK_SIZE,
				(void * 		) NULL,
				(UBaseType_t	) TASK1_TASK_PRIO,
				(TaskHandle_t*	) &Task1Task_Handler);
				
	//创建Task2
	xTaskCreate((TaskFunction_t	) task2_task,
							(char*			) "task2_task",
							(uint16_t		) TASK1_STK_SIZE,
							(void * 		) NULL,
							(UBaseType_t	) TASK2_TASK_PRIO,
							(TaskHandle_t*	) &Task2Task_Handler);

	vTaskStartScheduler();          //开启任务调度
}

void task1_task(void * pvParameters)
{
	u16 times=0;
	u32 SendCommand;
	BaseType_t xStatus;
	
  for(;;)
  {
			if(USART1_RX_STA&0x8000)//判断接收是否完成：0x8000=1000 0000 0000 0000/uint16_t USART1_RX_STA的bit15（接收完成标志）置1
			{	
				SendCommand=Get_USART_Command(&UART1_Handler);	
				xStatus=xQueueOverwrite(CommandQueue,&SendCommand);
				if(xStatus==pdPASS)
				{
					println_str(&UART1_Handler,"Send to the queue successfully!");
//					vTaskPrioritySet(Task2Task_Handler,3);
				}
				else
					println_str(&UART1_Handler,"Could not send to the queue!");
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

void task2_task(void * pvParameters)
{
	u32 ReceiveCommand=0;
	BaseType_t xStatus;
	const TickType_t xTicksToWait=pdMS_TO_TICKS(100UL);
	
	for(;;)
	{
		xStatus=xQueueReceive(CommandQueue,&ReceiveCommand,xTicksToWait);
		if(xStatus==pdPASS)
		{		
			taskENTER_CRITICAL();
			println_str(&UART1_Handler,"Receive from the queue successfully!");
			switch(ReceiveCommand)
			{
				case 1:
				{
					Wrist_Extension();
					break;
				}
				case 2:
				{
					Wrist_Flextion();
					break;
				}
				case 3:
				{
					Wrist_Ulnar_Deviation();
					break;
				}
				case 4:
				{
					Wrist_Radial_Deviation();
					break;
				}
				case 5:
				{
					HelloWorld();
					break;
				}
				case 6:
				{
					Motor_Reset();
					break;
				}
			}
//			vTaskPrioritySet(NULL,1);
			vTaskDelay(5);
			taskEXIT_CRITICAL();
		}
		else
			println_str(&UART1_Handler,"Could not receive from the queue!");
		ReceiveCommand=0;
	}
}


