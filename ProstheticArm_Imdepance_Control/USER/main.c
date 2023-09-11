#include "main.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"
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
#include "resistance.h"

/*任务*/
//创建开始任务（用于创建其他功能性任务）
#define START_TASK_PRIO			1
#define START_STK_SIZE			128
void start_task(void * pvParameters);
TaskHandle_t StartTask_Handler;		//任务句柄

//接收运动指令任务
#define COMMAND_TASK_PRIO			3
#define COMMAND_STK_SIZE			512
void Command_task(void * pvParameters);
TaskHandle_t CommandTask_Handler;		//任务句柄	 

//腕部位置传感器检测
#define WristPos_TASK_PRIO		2
#define WristPos_STK_SIZE			512
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
#define ITEM_SIZE       		sizeof(float)
QueueHandle_t WristCommandQueue=xQueueCreate(QUEUE_LENGTH,ITEM_SIZE);

//创建队列2：腕部姿态数据
#define WRISTPOS_QUEUE_LENGTH        3
#define WRISTPOS_ITEM_SIZE       		sizeof(float)
QueueHandle_t WristPosQueue=xQueueCreate(WRISTPOS_QUEUE_LENGTH,WRISTPOS_ITEM_SIZE);

//创建队列3:肘部运动指令
#define ELBOWCOMMAND_QUEUE_LENGTH        1
#define ELBOWCOMMAND_ITEM_SIZE       		sizeof(float)
QueueHandle_t ElbowCommandQueue=xQueueCreate(ELBOWCOMMAND_QUEUE_LENGTH,ELBOWCOMMAND_ITEM_SIZE);

//创建队列4：肘关节状态数据
#define ElbowKdata_QUEUE_LENGTH        4
#define ElbowKdata_ITEM_SIZE       		sizeof(float)
QueueHandle_t ElbowKdataQueue=xQueueCreate(ElbowKdata_QUEUE_LENGTH,ElbowKdata_ITEM_SIZE);

//互斥量句柄
SemaphoreHandle_t MutexSemaphore;

/*事件标志组*/
EventGroupHandle_t EventGroupHandler=xEventGroupCreate();

const TickType_t xTicksToWait=pdMS_TO_TICKS(100UL);

#define JACK 1

int main(void)
{
  HAL_Init();                    	//初始化HAL库    
  Stm32_Clock_Init(336,8,2,7);  	//设置时钟,168Mhz
	delay_init(168);               	//初始化延时函数
	MX_GPIO_Init();
	Serialport_Init(115200,115200,460800,115200,9600);              //初始化USART
	
	printf("System initiated!!  \r\n");
	
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
	
	//创建互斥量
	MutexSemaphore = xSemaphoreCreateMutex();
	
	//创建任务1：接收串口运动指令
	xTaskCreate((TaskFunction_t	) Command_task,
				      (char*			) "Command_task",
				      (uint16_t		) COMMAND_STK_SIZE,
				      (void * 		) NULL,
				      (UBaseType_t	) COMMAND_TASK_PRIO,
				      (TaskHandle_t*	) &CommandTask_Handler);
						
#if	WristPosReport						
  //创建Task3:检测腕部位置							
	xTaskCreate((TaskFunction_t	) WristPos_task,
							(char*			) "WristPos_task",
							(uint16_t		) WristPos_STK_SIZE,
							(void * 		) NULL,
							(UBaseType_t	) WristPos_TASK_PRIO,
							(TaskHandle_t*	) &WristPosTask_Handler);
#endif
							
#if JACK		
	//创建Task4:控制肘部的运动					
	xTaskCreate((TaskFunction_t	) Elbow_task,
							(char*			) "Elbow_task",
							(uint16_t		) ELBOW_STK_SIZE,
							(void * 		) NULL,
							(UBaseType_t	) ELBOW_TASK_PRIO,
							(TaskHandle_t*	) &ElbowTask_Handler);
#endif							
	vTaskDelete(StartTask_Handler); //删除开始任务
                
  taskEXIT_CRITICAL();            //退出临界区		
}


void Command_task(void * pvParameters)
{	
  for(;;)
  {	  
		if(xSemaphoreTake(MutexSemaphore,0) != 0) {
				taskENTER_CRITICAL();	//进入临界状态
				/*判断接收是否完成：0x8000=1000 0000 0000 0000/uint16_t USART1_RX_STA的bit15（接收完成标志）置1*/
				if((USART1_RX_STA & 0x8000) || (CAN1_RX_FLAG & 0x40)) {	
#if 	StopInterrupt
					portDISABLE_INTERRUPTS();
#endif
					int32_t SendCommand[14]={0};
					if(CAN1_RX_FLAG & 0x40)
						Get_CAN_Command(SendCommand);
					if(USART1_RX_STA&0x8000)
						Get_USART_Command(&UART1_Handler,SendCommand);	
				
//					WristPostureControl(SendCommand);//动平台姿态控制
//					Motor_QB(SendCommand[4]);
					Motor_ZB(SendCommand[6]);
//					nverse_kinematics(SendCommand[8],SendCommand[10],SendCommand[12]);

#if		StopInterrupt
				portENABLE_INTERRUPTS();
#endif
				}
				xSemaphoreGive(MutexSemaphore);
				taskEXIT_CRITICAL();	//退出临界状态
		}
		vTaskDelay(2);                                     
	}
}

void WristPos_task(void * pvParameters)
{
	float WristPos[2];
	BaseType_t SendStatus[3];
	u8 i;
	
  for(;;)
  {	
		taskENTER_CRITICAL();	//进入临界状态
		if(HAL_UART_Receive(&UART2_Handler, (u8 *)bRxBuffer,IMUFrameLength,20)==HAL_OK)
		{
			CopeMPData((unsigned char*)bRxBuffer);	
			
			printf("MPlatform:"
				   " X:%f  ,"
				   " Y:%f  ,"
				   " Z:%f  \r\n",
				   				MPlatform.hN_Yaw,
				   				MPlatform.hN_Pitch,
				   				MPlatform.hN_roll  );
		}

		if(HAL_UART_Receive(&UART4_Handler, (u8 *)dRxBuffer,IMUFrameLength,20)==HAL_OK)
		{
			CopeSPData((unsigned char*)dRxBuffer);
			printf("SPlatform:X:%f  , Y:%f  , Z:%f\r\n",SPlatform.hN_Yaw,SPlatform.hN_Pitch,SPlatform.hN_roll);
		}
		
		RPY(WristPos,SPlatform,MPlatform);//解算腕关节动平台相对于静平台的姿态
		printf("WristPos:%f  , %f  \r\n",WristPos[0]*180/Pi,WristPos[1]*180/Pi);
		taskEXIT_CRITICAL();	//退出临界状态
		
		vTaskDelay(100);//等待传输完成
  }
}

void Elbow_task(void * pvParameters)
{
	uint8_t flag_i = 0;
	int32_t last_pos = 0;		
	while(flag_i == 0)
	{
		flag_i = 1;

//		motor_ZB.Set_Operation_Mode(PPM);
//		motor_ZB.EnableDevice();
			Motor_ZB(0);
	}
	
	for(;;) {		
		if(xSemaphoreTake(MutexSemaphore,0) != 0) {
			taskENTER_CRITICAL();	//进入临界状态
			double ZB_torque = 0.00000;
			
			MotorData_CAN_Send(&ZB_torque);
			
			printf("/resistance insert/ ZB_torque = [%5.5f]\r\n", ZB_torque);
			printf("/resistance insert/ current pos=[%d] \r\n",*(int32_t *)(&data_ret[38]));
			printf("/resistance insert/ current vol=[%d] \r\n",*(int32_t *)(&data_ret[42]));
		
		
			if ( RET_TRUE == detect_torque_outside(ZB_torque, THREHOLD_TORQUE) ) {
					printf("/resistance insert/ |trigger on the touch!| \r\n");
					printf("/resistance insert/ |trigger on the touch!| current pos=[%d] \r\n",*(int32_t *)(&data_ret[38]));
					last_pos = *(int32_t *)(&data_ret[38]);
					printf("/resistance insert/ |trigger on the touch!| last_pos=[%d] \r\n",last_pos);
				
					//motor_ZB.MoveToPosition(0,80);
					motor_ZB.Set_Operation_Mode(CST);
					motor_ZB.Set_TargetTorque(0);
					motor_ZB.EnableDevice();
				
					while( 0 == *(int32_t *)(&data_ret[42]))//电机速度为0，处于静止状态
					{
							printf("/resistance insert/ |trigger on the touch!|while( 0 ==   \r\n");
							MotorData_CAN_Send(&ZB_torque);
					}
					
					while( 0 != *(int32_t *)(&data_ret[42]))    // Currently the force from outside is on here
					{
							
							printf("/resistance insert/ |trigger on the touch!|while( 0 !=   \r\n");
						
							MotorData_CAN_Send(&ZB_torque);
					}
					
					/*回到原位置*/
	//				motor_ZB.Set_Operation_Mode(PPM);
	//				motor_ZB.EnableDevice();
	//				motor_ZB.MoveToPosition(1,last_pos);
					printf("/resistance insert/ last_pos=[%d] \r\n",last_pos);
					
					motor_ZB.MoveToPosition(8000,10000,10000, 1, last_pos);
					
					while(!((( last_pos - 500 ) < *(int32_t *)(&data_ret[38])) && (( last_pos + 500 ) > *(int32_t *)(&data_ret[38]))))
					{
							MotorData_CAN_Send(&ZB_torque);
							printf("/resistance insert/ Havent came back to last position");
					}				
			}
			xSemaphoreGive(MutexSemaphore);
			taskEXIT_CRITICAL();	//退出临界状态
		}
		vTaskDelay(2);                                     
	}
}