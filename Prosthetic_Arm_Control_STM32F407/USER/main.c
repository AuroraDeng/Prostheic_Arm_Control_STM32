#include "main.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart_get.h"
#include "motioncontrol.h"
#include "motor_test.h"
#include "BWT61CL.h"

/*任务*/
//创建开始任务（用于创建其他功能性任务）
#define START_TASK_PRIO			1
#define START_STK_SIZE			128
void start_task(void * pvParameters);
TaskHandle_t StartTask_Handler;		//任务句柄

//接收运动指令任务
#define TASK1_TASK_PRIO			3
#define TASK1_STK_SIZE			128
void task1_task(void * pvParameters);
TaskHandle_t Task1Task_Handler;		//任务句柄	 

//腕部电机运动指令
#define WRIST_TASK_PRIO			2
#define WRIST_STK_SIZE			512
void Wrist_task(void * pvParameters);
TaskHandle_t WristTask_Handler;		//任务句柄

//腕部位置传感器检测
#define WristPos_TASK_PRIO			3
#define WristPos_STK_SIZE			256
void WristPos_task(void * pvParameters);
TaskHandle_t WristPosTask_Handler;		//任务句柄

//肘部电机运动指令
#define ELBOW_TASK_PRIO			2
#define ELBOW_STK_SIZE			512
void Elbow_task(void * pvParameters);
TaskHandle_t ElbowTask_Handler;		//任务句柄

/*消息队列*/
//创建队列1:腕部运动指令
#define QUEUE_LENGTH        3
#define ITEM_SIZE       		sizeof(int32_t)
QueueHandle_t CommandQueue=xQueueCreate(QUEUE_LENGTH,ITEM_SIZE);

//创建队列2：腕部姿态数据
#define WRISTPOS_QUEUE_LENGTH        3
#define WRISTPOS_ITEM_SIZE       		sizeof(float)
QueueHandle_t WristPosQueue=xQueueCreate(WRISTPOS_QUEUE_LENGTH,WRISTPOS_ITEM_SIZE);

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
	
	//创建任务1：接收串口运动指令
	xTaskCreate((TaskFunction_t	) task1_task,
				      (char*			) "task1_task",
				      (uint16_t		) TASK1_STK_SIZE,
				      (void * 		) NULL,
				      (UBaseType_t	) TASK1_TASK_PRIO,
				      (TaskHandle_t*	) &Task1Task_Handler);
				
	//创建Task2:控制腕部电机的运动
	xTaskCreate((TaskFunction_t	) Wrist_task,
							(char*			) "Wrist_task",
							(uint16_t		) WRIST_STK_SIZE,
							(void * 		) NULL,
							(UBaseType_t	) WRIST_TASK_PRIO,
							(TaskHandle_t*	) &WristTask_Handler);
							
  //创建Task3:检测腕部位置							
	xTaskCreate((TaskFunction_t	) WristPos_task,
							(char*			) "WristPos_task",
							(uint16_t		) WristPos_STK_SIZE,
							(void * 		) NULL,
							(UBaseType_t	) WristPos_TASK_PRIO,
							(TaskHandle_t*	) &WristPosTask_Handler);
//							
//	//创建Task4:控制电机3的运动					
//	xTaskCreate((TaskFunction_t	) Elbow_task,
//							(char*			) "Elbow_task",
//							(uint16_t		) ELBOW_STK_SIZE,
//							(void * 		) NULL,
//							(UBaseType_t	) ELBOW_TASK_PRIO,
//							(TaskHandle_t*	) &Motor3Task_Handler);
							
	vTaskDelete(StartTask_Handler); //删除开始任务
                
  taskEXIT_CRITICAL();            //退出临界区		
}

void task1_task(void * pvParameters)
{
	u16 times=0;
	BaseType_t SendStatus[3];
	u8 i;
	
  for(;;)
  {				
		if(USART1_RX_STA&0x8000)//判断接收是否完成：0x8000=1000 0000 0000 0000/uint16_t USART1_RX_STA的bit15（接收完成标志）置1
		{	
			taskENTER_CRITICAL();	//进入临界状态
			
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
			taskEXIT_CRITICAL();	//退出临界状态
		}
		else
		{
			times++;
			if(times%200==0)
			{
				taskENTER_CRITICAL();	//进入临界状态
				println_str(&UART1_Handler,"Please Enter the Prosthetic Arm Movement Command");
				taskEXIT_CRITICAL();	//退出临界状态
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
			/*获取假肢腕的实时姿态和目标姿态*/
			for(i=0;i<3;i++)
			{
				CommandStatus[i]=xQueueReceive(CommandQueue,&WristTargetPos[i],0);
				WristPosStatus[i]=xQueueReceive(WristPosQueue,&WristPos[i],0);
				PositionError[i]=(float)WristTargetPos[i]-WristPos[i];//假肢腕的实时位姿偏差
			}
		
			if(WristPosStatus[0]==pdPASS&&WristPosStatus[1]==pdPASS&&WristPosStatus[2]==pdPASS&&CommandStatus[0]==pdPASS&&CommandStatus[1]==pdPASS&&CommandStatus[2]==pdPASS)
			{
				println_str(&UART1_Handler,"Receive from the queue successfully!");

				taskENTER_CRITICAL();	//进入临界状态
				
				WristPositionControl(PositionError);
				
				taskEXIT_CRITICAL();	//退出临界状态
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
		taskENTER_CRITICAL();	//进入临界状态
		printf("-----------------------------------\r\n");
//		//输出加速度
//		//串口接受到的数据已经拷贝到对应的结构体的变量中了，根据说明书的协议，以加速度为例 stcAcc.a[0]/32768*16就是X轴的加速度，
//		printf("Acc:%.3f %.3f %.3f\r\n",(float)stcAcc.a[0]/32768*16,(float)stcAcc.a[1]/32768*16,(float)stcAcc.a[2]/32768*16);
//		vTaskDelay(10);
//		//输出角速度
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
		
		//输出角度
		printf("Angle:%.3f %.3f %.3f\r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180,(float)stcAngle.Angle[2]/32768*180);
		
		taskEXIT_CRITICAL();	//退出临界状态
//		vTaskDelay(1000);//等待传输完成
		vTaskDelay(10);//等待传输完成
  }
}

void Elbow_task(void * pvParameters)
{

}