#include "EPOS.h"

Epos::Epos()
{
}


/*设置EPOS运动模式*/
void Epos::Set_Operation_Mode(uint8_t mode)
{	
	Mode=mode;
	println_str(&UART1_Handler,"Set Operation Mode");
	while(ReceivePDO(Epos::NodeID,2,Epos::Mode)!=1);//将EPOS设置为控制模式
	delay_ms(10);
}

/*设置目标位置*/
void Epos::Set_TargetPosition(uint8_t IsAbsolute,int32_t target_position)
{	
	println_str(&UART1_Handler,"Set Target Position");
	ProfilePos=target_position;
	m_bIsAbsolute=IsAbsolute;
	
//	if(m_bIsAbsolute)
//		WriteControlword(absolute_movement);
//	else
//		WriteControlword(relative_movement);
	
	while(SDO_Write(Epos::NodeID,0x607A,0x00,Epos::ProfilePos)!=1);
	delay_ms(10);
}

void Epos::Set_ProfileVel(Uint32 profile_vel)
{
	println_str(&UART1_Handler,"Set Target Velocity");
	ProfileVel=profile_vel;
	
	while(ReceivePDO(Epos::NodeID,3,Epos::ProfileVel)!=1);
	delay_ms(10);
}

void Epos::Set_ProfileAcc(Uint32 profile_acc)
{
	println_str(&UART1_Handler,"Set Profile Acceleration");
	ProfileAcc=profile_acc;
	
	while(SDO_Write(Epos::NodeID,0x6083,0x00,Epos::ProfileAcc)!=1);
	delay_ms(10);
}

void Epos::Set_ProfileDec(Uint32 profile_dec)
{
	println_str(&UART1_Handler,"Set Profile Deceleration");
	ProfileDec=profile_dec;
	
	while(SDO_Write(Epos::NodeID,0x6084,0x00,Epos::ProfileDec)!=1);
	delay_ms(10);
}

void Epos::UpdateProfileData(Uint8 nodeid,Uint8 mode,int32_t profile_position,Uint32 profile_velocity,Uint32 profile_acceleration,Uint32 profile_deceleration,BOOL is_absolute)
{
	NodeID = nodeid;
	Mode= mode;
	ProfilePos = profile_position;
	ProfileVel = profile_velocity;
	ProfileAcc = profile_acceleration;
	ProfileDec = profile_deceleration;
	m_bIsAbsolute = is_absolute;
}

void Epos::UpdateActualData(int32_t actual_position,Uint32 actual_velocity)
{
	ActualPos = actual_position;
	ActualVel = actual_velocity;
}

void Epos::Get_ActualPos()
{	
	ActualPos=TransmitPDO(Epos::NodeID,3);
}

void Epos::Get_ActualVel()
{
	ActualVel=TransmitPDO(Epos::NodeID,2);
}

BOOL Epos::IsHomePosition()
{
	Get_ActualPos();
	if(ActualPos==0)
		return 1;
	else
		return 0;
}

void Epos::BackToHomePosition()
{
	MoveToPosition(ProfileVel,ProfileAcc,ProfileDec,1,0);
}

void Epos::MoveToPosition(Uint32 profile_vel,Uint32 profile_acc,Uint32 profile_dec,uint8_t IsAbsolute,int32_t target_position)
{
	Set_ProfileVel(profile_vel);
	Set_ProfileAcc(profile_acc);
	Set_ProfileDec(profile_dec);
	
	InitPPM();
	
	Set_TargetPosition(IsAbsolute,target_position);
	if(m_bIsAbsolute)
		WriteControlword(absolute_immediate_movement);
	else
		WriteControlword(relative_immediate_movement);
}

void Epos::MoveToPosition(Uint32 profile_vel,Uint32 profile_acc,Uint32 profile_dec,int32_t *posList, uint8_t len)
{	
	InitPPM();
	
	for(int i=0;i<len;i++)
	{
		MoveToPosition(profile_vel,profile_acc,profile_dec,1,posList[i]);
		//delay_ms(100);
		WriteControlword(0x000F);
	}
}

/*写入控制字*/
void Epos::WriteControlword(uint16_t controlword)
{
	while(ReceivePDO(Epos::NodeID,1,controlword)!=1);//RxPDO1传递控制字
	delay_ms(10);
}

/*读出状态字*/
uint16_t Epos::ReadStatusword()
{
	uint16_t statusword;
	statusword=TransmitPDO(Epos::NodeID,1);
	return statusword;
}

/*初始化轮廓位置控制模式PPM*/
void Epos::InitPPM()
{
	Set_Operation_Mode(PPM);
	
	println_str(&UART1_Handler,"Switch on and enable ...");
	
	WriteControlword(switchon_and_enable);//使能
	
	delay_ms(10);
}

/*初始化循环同步位置模式CSP*/
void Epos::InitCSP()
{
	Set_Operation_Mode(CSP);
	
	println_str(&UART1_Handler,"Switch on and enable ...");
	
	WriteControlword(switchon_and_enable);
	
	delay_ms(10);
}

