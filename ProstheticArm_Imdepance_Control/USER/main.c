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

//���������
SemaphoreHandle_t MutexSemaphore;

/*�¼���־��*/
EventGroupHandle_t EventGroupHandler=xEventGroupCreate();

const TickType_t xTicksToWait=pdMS_TO_TICKS(100UL);

#define JACK 1

int main(void)
{
  HAL_Init();                    	//��ʼ��HAL��    
  Stm32_Clock_Init(336,8,2,7);  	//����ʱ��,168Mhz
	delay_init(168);               	//��ʼ����ʱ����
	MX_GPIO_Init();
	Serialport_Init(115200,115200,460800,115200,9600);              //��ʼ��USART
	
	printf("System initiated!!  \r\n");
	
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
	
	//����������
	MutexSemaphore = xSemaphoreCreateMutex();
	
	//��������1�����մ����˶�ָ��
	xTaskCreate((TaskFunction_t	) Command_task,
				      (char*			) "Command_task",
				      (uint16_t		) COMMAND_STK_SIZE,
				      (void * 		) NULL,
				      (UBaseType_t	) COMMAND_TASK_PRIO,
				      (TaskHandle_t*	) &CommandTask_Handler);
						
#if	WristPosReport						
  //����Task3:�����λ��							
	xTaskCreate((TaskFunction_t	) WristPos_task,
							(char*			) "WristPos_task",
							(uint16_t		) WristPos_STK_SIZE,
							(void * 		) NULL,
							(UBaseType_t	) WristPos_TASK_PRIO,
							(TaskHandle_t*	) &WristPosTask_Handler);
#endif
							
#if JACK		
	//����Task4:�����ⲿ���˶�					
	xTaskCreate((TaskFunction_t	) Elbow_task,
							(char*			) "Elbow_task",
							(uint16_t		) ELBOW_STK_SIZE,
							(void * 		) NULL,
							(UBaseType_t	) ELBOW_TASK_PRIO,
							(TaskHandle_t*	) &ElbowTask_Handler);
#endif							
	vTaskDelete(StartTask_Handler); //ɾ����ʼ����
                
  taskEXIT_CRITICAL();            //�˳��ٽ���		
}


void Command_task(void * pvParameters)
{	
  for(;;)
  {	  
		if(xSemaphoreTake(MutexSemaphore,0) != 0) {
				taskENTER_CRITICAL();	//�����ٽ�״̬
				/*�жϽ����Ƿ���ɣ�0x8000=1000 0000 0000 0000/uint16_t USART1_RX_STA��bit15��������ɱ�־����1*/
				if((USART1_RX_STA & 0x8000) || (CAN1_RX_FLAG & 0x40)) {	
#if 	StopInterrupt
					portDISABLE_INTERRUPTS();
#endif
					int32_t SendCommand[14]={0};
					if(CAN1_RX_FLAG & 0x40)
						Get_CAN_Command(SendCommand);
					if(USART1_RX_STA&0x8000)
						Get_USART_Command(&UART1_Handler,SendCommand);	
				
//					WristPostureControl(SendCommand);//��ƽ̨��̬����
//					Motor_QB(SendCommand[4]);
					Motor_ZB(SendCommand[6]);
//					nverse_kinematics(SendCommand[8],SendCommand[10],SendCommand[12]);

#if		StopInterrupt
				portENABLE_INTERRUPTS();
#endif
				}
				xSemaphoreGive(MutexSemaphore);
				taskEXIT_CRITICAL();	//�˳��ٽ�״̬
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
		taskENTER_CRITICAL();	//�����ٽ�״̬
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
		
		RPY(WristPos,SPlatform,MPlatform);//������ؽڶ�ƽ̨����ھ�ƽ̨����̬
		printf("WristPos:%f  , %f  \r\n",WristPos[0]*180/Pi,WristPos[1]*180/Pi);
		taskEXIT_CRITICAL();	//�˳��ٽ�״̬
		
		vTaskDelay(100);//�ȴ��������
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
			taskENTER_CRITICAL();	//�����ٽ�״̬
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
				
					while( 0 == *(int32_t *)(&data_ret[42]))//����ٶ�Ϊ0�����ھ�ֹ״̬
					{
							printf("/resistance insert/ |trigger on the touch!|while( 0 ==   \r\n");
							MotorData_CAN_Send(&ZB_torque);
					}
					
					while( 0 != *(int32_t *)(&data_ret[42]))    // Currently the force from outside is on here
					{
							
							printf("/resistance insert/ |trigger on the touch!|while( 0 !=   \r\n");
						
							MotorData_CAN_Send(&ZB_torque);
					}
					
					/*�ص�ԭλ��*/
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
			taskEXIT_CRITICAL();	//�˳��ٽ�״̬
		}
		vTaskDelay(2);                                     
	}
}