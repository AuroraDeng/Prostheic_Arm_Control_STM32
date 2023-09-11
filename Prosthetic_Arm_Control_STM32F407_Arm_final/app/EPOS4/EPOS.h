#ifndef __EPOS_H
#define __EPOS_H 	

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "can.h"
#include "pdo.h"
#include "sdo.h"
#include "NMT_control.h"
#include "usart_print.h"
#include "delay.h"
#include "sys.h"
#include "main.h"

//------------------------------------------------------------------------
//define variables  �����������
//------------------------------------------------------------------------
#ifdef __cplusplus
 extern "C" {
#endif
	 
typedef char                    int8;
typedef int                     int16;
typedef long                    int32;
typedef uint8_t                 Uint8;
typedef uint16_t                Uint16;
typedef uint32_t                Uint32;
typedef uint8_t                 BOOL;
typedef uint16_t                stcCAN;
	 
class Epos
{
	public:		
		
		Uint8 NodeID;
		Uint8 Mode;
		//Uint32 error_window;
		int32_t ProfilePos;//Ŀ��λ��
		int32_t ActualPos;//ʵ��λ��
		Uint32 ProfileVel;//����ת��
		int32_t ActualVel;//ʵ��ת��
		Uint32 ProfileAcc;//���ü��ٶ�
		Uint32 ActualAcc;//ʵ�ʼ��ٶ�
	
		Uint32 ProfileDec;//���ü��ٶ�
		double AcutalTorque;
		BOOL  m_bIsAbsolute; //�Ƿ�Ϊ�����˶�
	
		/*��ʼ��*/
		Epos(  
			 Uint8 nodeid,							// 0
			 Uint8 mode,
			 int32_t profile_pos=0,			// 2
			 int32_t actual_pos =0,
			 Uint32 profile_vel =3000,  // 4
			 Uint32 actual_vel  =0,
			 Uint32 profile_acc =10000,
			 Uint32 actual_acc  =0,     // 7
			 Uint32 profile_dec =10000,
			 double acutaltorque=0,
			 BOOL   isabsolute  =1      // 10
		):
			 NodeID(nodeid),						// 0
			 Mode(mode),
			 ProfilePos(profile_pos),   // 2
			 ActualPos(actual_pos),
			 ProfileVel(profile_vel),   // 4
			 ActualVel(actual_vel),
			 ProfileAcc(profile_acc),
			 ActualAcc(actual_acc),			// 7
			 ProfileDec(profile_dec),
			 m_bIsAbsolute(isabsolute),
			 AcutalTorque(acutaltorque) // 10
				{ }
		Epos();
		
		void InitPPM();
		void InitCSP();
		void EnableDevice();
		/*д��&��ȡ����*/
		void WriteControlword(uint16_t controlword);
		uint16_t ReadStatusword();
		/*д��&��ȡ�����ֵ�*/
		void WriteObject(uint16_t Index,uint8_t SubIndex,uint32_t param);
		/*���ò���*/
		void Set_Operation_Mode(uint8_t mode);
		void Set_TargetPosition(uint8_t IsAbsolute,int32_t target_position);//������
		void Set_ProfileVel(Uint32 profile_vel);
		void Set_ProfileAcc(Uint32 profile_acc);
		void Set_ProfileDec(Uint32 profile_dec);
		void Set_TargetTorque(int16_t TargetTorque);
		void UpdateProfileData(Uint8 nodeid,
													 Uint8 mode,
													 int32_t profile_position,
													 Uint32 profile_velocity,
												   Uint32 profile_acceleration,
													 Uint32 profile_deceleration,
													 BOOL is_absolute           );
		void UpdateActualData(int32_t actual_position,Uint32 actual_velocity);
		/*��ȡ�˶���Ϣ*/
		int32_t Get_ActualPos();
		void Get_ActualPos(uint8_t pos_data[]);
		double Get_ActualVel(); 
		void Get_ActualVel(uint8_t vel_data[]);
		int32_t Get_ActualAcc();
		double Get_AcutalTor();
		void Get_ActualCurrent(uint8_t cur_data[]);
		BOOL IsHomePosition();
	
		void MoveToPosition(Uint32 profile_vel,
												Uint32 profile_acc,
												Uint32 profile_dec,
												uint8_t IsAbsolute,
												int32_t target_position);
		void MoveToPosition(uint8_t IsAbsolute,int32_t target_position);
		void MoveToPosition(Uint32 profile_vel,
												Uint32 profile_acc,
												Uint32 profile_dec,
												int32_t *posList, 
												uint8_t len);
		void BackToHomePosition();
		
		void ClearFault();//���
};

/**
* @addtogroup DeviceControlCommands
* @brief Device Control Commands
* @details For more details look in the EPOS4 Firmware Specification - 2.2 Device Control
* @{
*/
		
enum epos_controlword
{
	shutdown = 0x06,            /**< will end in device state >ready to switch on< */
  switchon = 0x07,            /**< will end in device state >switched on< */
  switchon_and_enable = 0x0F, /**< will end in device state >switch on< then    
																			auto transition to >operation enabled< */
  disablevoltage = 0x00,      /**< will end in device state >switch on disabled< */
  quickstop = 0x02,           /**< will end in device state >quick stop active< */
  disableoperation = 0x07,    /**< will end in device state >switched on< */
  enableoperation = 0x0F,     /**< will end in device state >operation enabled< */
  faultreset = 0x80,          /**< will end in device state >Fault< and/or > 
																		switch on disabled< */
	absolute_movement = 0x1F,
	relative_movement = 0x5F,
	absolute_immediate_movement = 0x3F,
	relative_immediate_movement =0x7F
};

enum epos_mode
{
	PPM = 0x01,            			/*����λ�ÿ���ģʽ*/
  HHM = 0x06,            			/*����ģʽ*/
  PVM = 0x03, 								/*�����ٶȿ���ģʽ*/
  CSP = 0x08,      						/*ѭ��ͬ��λ��ģʽ*/
  CSV = 0x09,           			/*ѭ��ͬ���ٶ�ģʽ*/
  CST = 0x0A,    							/*ѭ��ͬ������ģʽ*/
};

extern Epos motor_W1;
extern Epos motor_W2;
extern Epos motor_QB;
extern Epos motor_ZB;

#ifdef __cplusplus
}
#endif
#endif
