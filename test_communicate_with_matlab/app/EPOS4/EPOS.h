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

//------------------------------------------------------------------------
//define variables  定义变量类型
//------------------------------------------------------------------------

typedef char                    int8;
typedef int                     int16;
typedef long                    int32;
typedef uint8_t                 Uint8;
typedef uint16_t                Uint16;
typedef uint32_t                Uint32;
typedef uint8_t                 BOOL;
typedef uint16_t                stcCAN;

#ifdef __cplusplus
 extern "C" {
#endif
	 
class Epos
{
	public:		
		
		Uint8 NodeID;
		Uint8 Mode;
		//Uint32 error_window;
		int32_t ProfilePos;//目标位置
		int32_t ActualPos;//实际位置
		Uint32 ProfileVel;//设置转速
		Uint32 ActualVel;//实际转速
		Uint32 ProfileAcc;//设置加速度
		Uint32 ProfileDec;//设置减速度
		BOOL  m_bIsAbsolute; //是否为绝对运动
	
		/*初始化*/
		Epos(Uint8 nodeid,Uint8 mode,int32_t profile_pos=0,int32_t actual_pos=0,Uint32 profile_vel=1000,Uint32 actual_vel=0,Uint32 profile_acc=10000,Uint32 profile_dec=10000,BOOL isabsolute=1):
		NodeID(nodeid),Mode(mode),ProfilePos(profile_pos),ActualPos(actual_pos),ProfileVel(profile_vel),ActualVel(actual_vel),ProfileAcc(profile_acc),ProfileDec(profile_dec),m_bIsAbsolute(isabsolute)
		{ }
		Epos();
		
		void InitPPM();
		void InitCSP();
		/*写入&读取命令*/
		void WriteControlword(uint16_t controlword);
		uint16_t ReadStatusword();
		/*设置参数*/
		void Set_Operation_Mode(uint8_t mode);
		void Set_TargetPosition(uint8_t IsAbsolute,int32_t target_position);//？？？
		void Set_ProfileVel(Uint32 profile_vel);
		void Set_ProfileAcc(Uint32 profile_acc);
		void Set_ProfileDec(Uint32 profile_dec);
		void UpdateProfileData(Uint8 nodeid,Uint8 mode,int32_t profile_position,Uint32 profile_velocity,Uint32 profile_acceleration,Uint32 profile_deceleration,BOOL is_absolute);
		void UpdateActualData(int32_t actual_position,Uint32 actual_velocity);
		/*获取运动信息*/
		void Get_ActualPos();
		void Get_ActualVel(); 
		BOOL IsHomePosition();
	
		void MoveToPosition(Uint32 profile_vel,Uint32 profile_acc,Uint32 profile_dec,uint8_t IsAbsolute,int32_t target_position);
		void MoveToPosition(Uint32 profile_vel,Uint32 profile_acc,Uint32 profile_dec,int32_t *posList, uint8_t len);
		void BackToHomePosition();
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
  switchon_and_enable = 0x0F, /**< will end in device state >switch on< then auto transition to >operation enabled< */
  disablevoltage = 0x00,      /**< will end in device state >switch on disabled< */
  quickstop = 0x02,           /**< will end in device state >quick stop active< */
  disableoperation = 0x07,    /**< will end in device state >switched on< */
  enableoperation = 0x0F,     /**< will end in device state >operation enabled< */
  faultreset = 0x80,          /**< will end in device state >Fault< and/or >switch on disabled< */
	absolute_movement = 0x1F,
	relative_movement = 0x5F,
	absolute_immediate_movement = 0x3F,
	relative_immediate_movement =0x7F
};

enum epos_mode
{
	PPM = 0x01,            			/*轮廓位置控制模式*/
  HHM = 0x06,            			/*归零模式*/
  PVM = 0x03, 								/*轮廓速度控制模式*/
  CSP = 0x08,      						/*循环同步位置模式*/
  CSV = 0x09,           			/*循环同步速度模式*/
  CST = 0x0A,    							/*循环同步力矩模式*/
};

#ifdef __cplusplus
}
#endif
#endif
