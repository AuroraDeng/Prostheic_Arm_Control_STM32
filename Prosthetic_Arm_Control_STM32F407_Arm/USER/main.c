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
#include "BWT61CL.h"
#include "KWR46C.h"
#include "HWT101.h"
#include "kaerman.h"

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

//腕部电机运动指令
#define WRIST_TASK_PRIO			3
#define WRIST_STK_SIZE			512
void Wrist_task(void * pvParameters);
TaskHandle_t WristTask_Handler;		//任务句柄

//腕部位置传感器检测
#define WristPos_TASK_PRIO		2
#define WristPos_STK_SIZE			256
void WristPos_task(void * pvParameters);
TaskHandle_t WristPosTask_Handler;		//任务句柄

//肘部电机运动指令
#define ELBOW_TASK_PRIO			2
#define ELBOW_STK_SIZE			512
void Elbow_task(void * pvParameters);
TaskHandle_t ElbowTask_Handler;		//任务句柄

//肘部力矩及运动状态检测
#define ElbowStatus_TASK_PRIO		2
#define ElbowStatus_STK_SIZE			256
void ElbowStatus_task(void * pvParameters);
TaskHandle_t ElbowStatusTask_Handler;		//任务句柄

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

/*事件标志组*/
EventGroupHandle_t EventGroupHandler=xEventGroupCreate();

const TickType_t xTicksToWait=pdMS_TO_TICKS(100UL);

int main(void)
{
  HAL_Init();                    	//初始化HAL库    
  Stm32_Clock_Init(336,8,2,7);  	//设置时钟,168Mhz
	delay_init(168);               	//初始化延时函数
	MX_GPIO_Init();
	Serialport_Init(115200,115200,460800,115200,9600);              //初始化USART
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
	xTaskCreate((TaskFunction_t	) Command_task,
				      (char*			) "Command_task",
				      (uint16_t		) COMMAND_STK_SIZE,
				      (void * 		) NULL,
				      (UBaseType_t	) COMMAND_TASK_PRIO,
				      (TaskHandle_t*	) &CommandTask_Handler);
				
//	//创建Task2:控制腕部电机的运动
//	xTaskCreate((TaskFunction_t	) Wrist_task,
//							(char*			) "Wrist_task",
//							(uint16_t		) WRIST_STK_SIZE,
//							(void * 		) NULL,
//							(UBaseType_t	) WRIST_TASK_PRIO,
//							(TaskHandle_t*	) &WristTask_Handler);
//							
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
//							(TaskHandle_t*	) &ElbowTask_Handler);

//  //创建Task5:检测肘关节力矩及运动状态							
//	xTaskCreate((TaskFunction_t	) ElbowStatus_task,
//							(char*			) "ElbowStatus_task",
//							(uint16_t		) ElbowStatus_STK_SIZE,
//							(void * 		) NULL,
//							(UBaseType_t	) ElbowStatus_TASK_PRIO,
//							(TaskHandle_t*	) &ElbowStatusTask_Handler);
							
	vTaskDelete(StartTask_Handler); //删除开始任务
                
  taskEXIT_CRITICAL();            //退出临界区		
}

void Command_task(void * pvParameters)
{	
	float WristPos[3]={0};
	float delta_L[2]={0};
	int32_t x,y;
	int32_t M1=0,M2=0;
  for(;;)
  {	
		taskENTER_CRITICAL();	//进入临界状态
		
//		if(CAN1_RX_FLAG & 0x40)//判断接收是否完成：0x20=0010 0000/uint8_t USART1_RX_STA的bit6（接收完成标志）置1、
		if((USART1_RX_STA&0x8000)||(CAN1_RX_FLAG & 0x40))//判断接收是否完成：0x8000=1000 0000 0000 0000/uint16_t USART1_RX_STA的bit15（接收完成标志）置1
		{	
			int32_t SendCommand[14]={0};
			if(CAN1_RX_FLAG & 0x40)
				Get_CAN_Command(SendCommand);
			if(USART1_RX_STA&0x8000)
				Get_USART_Command(&UART1_Handler,SendCommand);	
			
//			MPlatform.Angle_Calcu();
//			SPlatform.Angle_Calcu();
//			RPY(WristPos,SPlatform,MPlatform);//解算腕关节动平台相对于静平台的姿态(输出是弧度)
			
			/*绳子的缩短量计算*/
			delta_L[0]=sqrt(2536*(1-cos((betax-abs(SendCommand[0]))*Pi/180)))-sqrt(2536*(1-cos(betax*Pi/180-abs(WristPos[0]))));//IMU的X轴转动/尺偏桡偏运动/左右方向
			delta_L[1]=sqrt(2320*(1-cos((betay-abs(SendCommand[1]))*Pi/180)))-sqrt(2320*(1-cos(betay*Pi/180-abs(WristPos[1]))));//IMU的Y轴转动/屈曲伸展运动/前后方向
			/*姿态引起的电机形成行程的绝对值*/
			x=((abs(delta_L[0])/R_CL)*(180/Pi))/((360/GearRatio)/(CountsPerTurn*Harmonic));
			y=((abs(delta_L[1])/R_CL)*(180/Pi))/((360/GearRatio)/(CountsPerTurn*Harmonic));
			/*电机行程计算*/
			M1=0;M2=0;
			if(SendCommand[0]>0)//右偏
			{
				M1+=x;
				M2+=x;
			}
			if(SendCommand[0]<0)//左偏
			{
				M1-=x;
				M2-=x;
			}
			if(SendCommand[1]<0)//向前
			{
				M1-=y;
				M2+=y;
			}
			if(SendCommand[1]>0)//向后
			{
				M1+=y;
				M2-=y;
			}
			Motor_W1(M1);
			Motor_W2(M2);
			Motor_QB(SendCommand[4]-50000);
			Motor_ZB(SendCommand[6]);
			
			MotorData_CAN_Send();
		}
		taskEXIT_CRITICAL();	//退出临界状态
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
//				/*获取假肢腕的实时姿态和目标姿态*/
//				for(i=0;i<3;i++)
//				{
//					CommandStatus[i]=xQueueReceive(WristCommandQueue,&WristTargetPos[i],0);
//					WristPosStatus[i]=xQueueReceive(WristPosQueue,&WristPos[i],0);
//					if(WristPosStatus[i]==pdPASS&&CommandStatus[i]==pdPASS)
//					{					
//						println_str(&UART1_Handler,"Receive from the queue successfully!");
//						PositionError[i]=WristTargetPos[i]-WristPos[i];//假肢腕的实时位姿偏差
//					}
//				}
//				taskENTER_CRITICAL();	//进入临界状态	
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
//				taskEXIT_CRITICAL();	//退出临界状态
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
		taskENTER_CRITICAL();	//进入临界状态
//		if(HAL_UART_Receive(&UART2_Handler, (u8 *)bRxBuffer,IMUFrameLength,20)==HAL_OK)
//		{
//			CopeMPData((unsigned char*)bRxBuffer);
			printf("MPlatform:%f  , %f  , %f\r\n",MPlatform.hN_Yaw,MPlatform.hN_Pitch,MPlatform.hN_roll);
//		}
		
//		if(HAL_UART_Receive(&UART4_Handler, (u8 *)dRxBuffer,IMUFrameLength,20)==HAL_OK)
//		{
//			CopeSPData((unsigned char*)dRxBuffer);
			printf("SPlatform:%f  , %f  , %f\r\n",SPlatform.hN_Yaw,SPlatform.hN_Pitch,SPlatform.hN_roll);
//		}
		
		RPY(WristPos,SPlatform,MPlatform);//解算腕关节动平台相对于静平台的姿态
		printf("WristPos:%f  , %f  \r\n",WristPos[0],WristPos[1]);
		
		taskEXIT_CRITICAL();	//退出临界状态
		
		vTaskDelay(100);//等待传输完成
  }
}

//void Elbow_task(void * pvParameters)
//{
//	
//	BaseType_t CommandStatus;
//	for(;;)
//	{
//		taskENTER_CRITICAL();	//进入临界状态
//		if(xEventGroupWaitBits(EventGroupHandler,0x02,pdTRUE,pdTRUE,0)&0x02)
//		{
//			float ReceiveCommand=0;
//			CommandStatus=xQueueReceive(ElbowCommandQueue,&ReceiveCommand,0);
//			if(CommandStatus==pdPASS)
//				Elbow_CompliantControl(ReceiveCommand);
//			else
//				println_str(&UART1_Handler,"Could not Receive Elbow Command!");
//		}
//		taskEXIT_CRITICAL();	//退出临界状态
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
//	WitInit(WIT_PROTOCOL_NORMAL, 0x50);//初始化角度传感器标准协议，设置设备地址(为什么设置成0x50)
//	WitSerialWriteRegister(SendCmdtoHWT);//注册写回调函数
//	WitRegisterCallBack(SensorDataUpdata);//注册获取传感器数据回调函数（更新数据状态：表示传感数据已经回传）
//	WitDelayMsRegister(delay_ms);
//	AutoScanSensor();

//	for(;;)
//	{
//		taskENTER_CRITICAL();	//进入临界状态
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
//		taskEXIT_CRITICAL();	//退出临界状态
//			
//		vTaskDelay(100);//等待传输完成
//	}
//}