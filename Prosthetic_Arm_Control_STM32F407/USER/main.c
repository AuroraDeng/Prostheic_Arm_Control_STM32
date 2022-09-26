#include "main.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart_get.h"
#include "motioncontrol.h"
#include "motor_test.h"
#include "usart_host_communication.h"

//创建开始任务（用于创建其他功能性任务）
#define START_TASK_PRIO			1
#define START_STK_SIZE			128
void start_task(void * pvParameters);
TaskHandle_t StartTask_Handler;		//任务句柄

//接收运动指令任务
#define TASK1_TASK_PRIO			1
#define TASK1_STK_SIZE			128
void task1_task(void * pvParameters);
TaskHandle_t Task1Task_Handler;		//任务句柄	 

//电机1运动指令
#define MOTOR1_TASK_PRIO			1
#define MOTOR1_STK_SIZE			256
void motor1_task(void * pvParameters);
TaskHandle_t Motor1Task_Handler;		//任务句柄

//电机2运动指令
#define MOTOR2_TASK_PRIO			1
#define MOTOR2_STK_SIZE			256
void motor2_task(void * pvParameters);
TaskHandle_t Motor2Task_Handler;		//任务句柄

//电机3运动指令
#define MOTOR3_TASK_PRIO			1
#define MOTOR3_STK_SIZE			256
void motor3_task(void * pvParameters);
TaskHandle_t Motor3Task_Handler;		//任务句柄

//创建队列
#define QUEUE_LENGTH        3
#define ITEM_SIZE       		sizeof(uint32_t)
QueueHandle_t CommandQueue=xQueueCreate(QUEUE_LENGTH,ITEM_SIZE);

int main(void)
{
  HAL_Init();                    	//初始化HAL库    
  Stm32_Clock_Init(336,8,2,7);  	//设置时钟,168Mhz
	delay_init(168);               	//初始化延时函数
	Serialport_Init(115200,115200);              //初始化USART
	CAN_Config();

	//创建开始任务
   xTaskCreate((TaskFunction_t )start_task,            //任务函数
               (const char*    )"start_task",          //任务名称
               (uint16_t       )START_STK_SIZE,        //任务堆栈大小
               (void*          )NULL,                  //传递给任务函数的参数
               (UBaseType_t    )START_TASK_PRIO,       //任务优先级
               (TaskHandle_t*  )&StartTask_Handler);   //任务句柄  

	vTaskStartScheduler();          //开启任务调度
}

void start_task(void * pvParameters)
{
	taskENTER_CRITICAL();           //进入临界区
	
	//创建任务1：接收串口运动指令
	xTaskCreate((TaskFunction_t	) task1_task,
				      (char*			) "task1_task",
				      (uint16_t		) TASK1_STK_SIZE,
				      (void * 		) NULL,
				      (UBaseType_t	) TASK1_TASK_PRIO,
				      (TaskHandle_t*	) &Task1Task_Handler);
				
	//创建Task2:控制电机1的运动
	xTaskCreate((TaskFunction_t	) motor1_task,
							(char*			) "motor1_task",
							(uint16_t		) MOTOR1_STK_SIZE,
							(void * 		) NULL,
							(UBaseType_t	) MOTOR1_TASK_PRIO,
							(TaskHandle_t*	) &Motor1Task_Handler);
							
  //创建Task3:控制电机2的运动					
	xTaskCreate((TaskFunction_t	) motor2_task,
							(char*			) "motor2_task",
							(uint16_t		) MOTOR2_STK_SIZE,
							(void * 		) NULL,
							(UBaseType_t	) MOTOR2_TASK_PRIO,
							(TaskHandle_t*	) &Motor2Task_Handler);
							
	//创建Task4:控制电机3的运动					
	xTaskCreate((TaskFunction_t	) motor3_task,
							(char*			) "motor3_task",
							(uint16_t		) MOTOR3_STK_SIZE,
							(void * 		) NULL,
							(UBaseType_t	) MOTOR3_TASK_PRIO,
							(TaskHandle_t*	) &Motor3Task_Handler);
							
	vTaskDelete(StartTask_Handler); //删除开始任务
                
  taskEXIT_CRITICAL();            //退出临界区		
}

void task1_task(void * pvParameters)
{
	u16 times=0;
	u32 SendCommand[3];
	BaseType_t xStatus;
	
  for(;;)
  {
			if(USART1_RX_STA&0x8000)//判断接收是否完成：0x8000=1000 0000 0000 0000/uint16_t USART1_RX_STA的bit15（接收完成标志）置1
			{	
				Get_USART_Command(&UART1_Handler,SendCommand);	
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

void motor1_task(void * pvParameters)
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

void motor2_task(void * pvParameters)
{

}

void motor3_task(void * pvParameters)
{

}

