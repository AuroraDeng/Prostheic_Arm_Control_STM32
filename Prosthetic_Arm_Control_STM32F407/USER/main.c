#include "main.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart_get.h"
#include "motioncontrol.h"
#include "motor_test.h"
#include "BWT61CL.h"

//创建开始任务（用于创建其他功能性任务）
#define START_TASK_PRIO			1
#define START_STK_SIZE			128
void start_task(void * pvParameters);
TaskHandle_t StartTask_Handler;		//任务句柄

//接收运动指令任务
#define TASK1_TASK_PRIO			4
#define TASK1_STK_SIZE			128
void task1_task(void * pvParameters);
TaskHandle_t Task1Task_Handler;		//任务句柄	 

//电机运动指令
#define MOTOR_TASK_PRIO			2
#define MOTOR_STK_SIZE			512
void motor_task(void * pvParameters);
TaskHandle_t MotorTask_Handler;		//任务句柄

//腕部位置传感器检测
#define WristPos_TASK_PRIO			3
#define WristPos_STK_SIZE			128
void WristPos_task(void * pvParameters);
TaskHandle_t WristPosTask_Handler;		//任务句柄

////电机3运动指令
//#define MOTOR3_TASK_PRIO			2
//#define MOTOR3_STK_SIZE			512
//void motor3_task(void * pvParameters);
//TaskHandle_t Motor3Task_Handler;		//任务句柄

//创建队列
#define QUEUE_LENGTH        3
#define ITEM_SIZE       		sizeof(int32_t)
QueueHandle_t CommandQueue=xQueueCreate(QUEUE_LENGTH,ITEM_SIZE);

const TickType_t xTicksToWait=pdMS_TO_TICKS(100UL);

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
	
//	//创建任务1：接收串口运动指令
//	xTaskCreate((TaskFunction_t	) task1_task,
//				      (char*			) "task1_task",
//				      (uint16_t		) TASK1_STK_SIZE,
//				      (void * 		) NULL,
//				      (UBaseType_t	) TASK1_TASK_PRIO,
//				      (TaskHandle_t*	) &Task1Task_Handler);
//				
//	//创建Task2:控制腕部电机的运动
//	xTaskCreate((TaskFunction_t	) motor_task,
//							(char*			) "motor_task",
//							(uint16_t		) MOTOR_STK_SIZE,
//							(void * 		) NULL,
//							(UBaseType_t	) MOTOR_TASK_PRIO,
//							(TaskHandle_t*	) &MotorTask_Handler);
							
  //创建Task3:检测腕部位置							
	xTaskCreate((TaskFunction_t	) WristPos_task,
							(char*			) "WristPos_task",
							(uint16_t		) WristPos_STK_SIZE,
							(void * 		) NULL,
							(UBaseType_t	) WristPos_TASK_PRIO,
							(TaskHandle_t*	) &WristPosTask_Handler);
//							
//	//创建Task4:控制电机3的运动					
//	xTaskCreate((TaskFunction_t	) motor3_task,
//							(char*			) "motor3_task",
//							(uint16_t		) MOTOR3_STK_SIZE,
//							(void * 		) NULL,
//							(UBaseType_t	) MOTOR3_TASK_PRIO,
//							(TaskHandle_t*	) &Motor3Task_Handler);
							
	vTaskDelete(StartTask_Handler); //删除开始任务
                
  taskEXIT_CRITICAL();            //退出临界区		
}

void task1_task(void * pvParameters)
{
	u16 times=0;
	BaseType_t SendStatus;
	u8 i;
	
  for(;;)
  {				
		if(USART1_RX_STA&0x8000)//判断接收是否完成：0x8000=1000 0000 0000 0000/uint16_t USART1_RX_STA的bit15（接收完成标志）置1
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

void WristPos_task(void * pvParameters)
{
	u8 i=0;
  for(;;)
  {				
		vTaskDelay(1000);
		i++;
		if(i>10)
		{
			i = 0;
			printf("Acceleration calibration is in progress.\r\n");
			SendCmdtoBWT(ACCCMD);//等待模块内部自动校准好，模块内部会自动计算需要一定的时间
			printf("Acceleration calibration is complete.\r\n");
			delay_ms(100);
			printf("Perform Z-axis angle clearance.\r\n");
			SendCmdtoBWT(YAWCMD);
			printf("The Z-axis angle is cleared to zero.\r\n");
		}
		printf("-----------------------------------\r\n");
		//输出加速度
		//串口接受到的数据已经拷贝到对应的结构体的变量中了，根据说明书的协议，以加速度为例 stcAcc.a[0]/32768*16就是X轴的加速度，
//		printf("Acc:%.3f %.3f %.3f\r\n",(float)stcAcc.a[0]/32768*16,(float)stcAcc.a[1]/32768*16,(float)stcAcc.a[2]/32768*16);
//		vTaskDelay(10);
//		//输出角速度
//		printf("Gyro:%.3f %.3f %.3f\r\n",(float)stcGyro.w[0]/32768*2000,(float)stcGyro.w[1]/32768*2000,(float)stcGyro.w[2]/32768*2000);
//		vTaskDelay(10);
		//输出角度
		printf("Angle:%.3f %.3f %.3f\r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180,(float)stcAngle.Angle[2]/32768*180);
		vTaskDelay(10);//等待传输完成
  }
}