#include "EPOS.h"

Epos motor_W1(1,PPM);
Epos motor_W2(2,PPM);
Epos motor_QB(3,PPM);
Epos motor_ZB(4,PPM);

Epos::Epos()
{
}


/*设置EPOS运动模式*/
void Epos::Set_Operation_Mode(uint8_t mode)
{	
	Mode=mode;
	println_str(&UART1_Handler,"Set Operation Mode");
	while(ReceivePDO(Epos::NodeID,2,Epos::Mode)!=1)//将EPOS设置为控制模式
	{delay_ms(2);}
}

/*设置目标位置*/
void Epos::Set_TargetPosition(uint8_t IsAbsolute,int32_t target_position)
{	
	println_str(&UART1_Handler,"Set Target Position");
	ProfilePos=target_position;
	m_bIsAbsolute=IsAbsolute;
	
	while(SDO_Write(Epos::NodeID,0x607A,0x00,Epos::ProfilePos)!=1)
	{delay_ms(2);}
}

/*设置目标力矩*/
void Epos::Set_TargetTorque(int16_t TargetTorque)
{
	println_str(&UART1_Handler,"Set Target Torque");
	if(Get_AcutalTor()!=TargetTorque)
		
	while(ReceivePDO(NodeID,3,TargetTorque)!=1)
	{delay_ms(2);}
}

void Epos::Set_ProfileVel(Uint32 profile_vel)
{
	println_str(&UART1_Handler,"Set Target Velocity");
	ProfileVel=profile_vel;
	
	while(ReceivePDO(Epos::NodeID,3,Epos::ProfileVel)!=1)
	{delay_ms(2);}
}

void Epos::Set_ProfileAcc(Uint32 profile_acc)
{
	println_str(&UART1_Handler,"Set Profile Acceleration");
	ProfileAcc=profile_acc;
	
	while(SDO_Write(Epos::NodeID,0x6083,0x00,Epos::ProfileAcc)!=1)
	{delay_ms(2);}
}

void Epos::Set_ProfileDec(Uint32 profile_dec)
{
	println_str(&UART1_Handler,"Set Profile Deceleration");
	ProfileDec=profile_dec;
	
	while(SDO_Write(Epos::NodeID,0x6084,0x00,Epos::ProfileDec)!=1)
	{delay_ms(2);}
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

int32_t Epos::Get_ActualPos()
{	
//	WriteControlword(switchon_and_enable);
	uint8_t pos_data[4];
	TransmitPDO(Epos::NodeID,3,pos_data);
	ActualPos=(pos_data[3]<<24)+(pos_data[2]<<16)+(pos_data[1]<<8)+pos_data[0];
	
	print_str(&UART1_Handler,"The motor ");
	print_data_dec(&UART1_Handler,Epos::NodeID);
	print_str(&UART1_Handler,"'s actual position is at ");
	print_data_dec(&UART1_Handler,Epos::ActualPos);
	print_str(&UART1_Handler,"\n");
	return ActualPos;
	
}
void Epos::Get_ActualPos(uint8_t pos_data[])
{
	TransmitPDO(Epos::NodeID,3,pos_data);
}

double Epos::Get_AcutalTor()
{
	uint8_t tor_data[2];
	TransmitPDO(Epos::NodeID,4,tor_data);
	int16_t act_tor=(tor_data[1]<<8)+tor_data[0];
	AcutalTorque=((double)act_tor/1000.0)*0.106909;//Nm
	return AcutalTorque;
}

double Epos::Get_ActualVel()
{
	uint8_t vel_data[4];
	TransmitPDO(Epos::NodeID,2,vel_data);
	ActualVel=(vel_data[3]<<24)+(vel_data[2]<<16)+(vel_data[1]<<8)+vel_data[0];//mrpm
	double dupers=(0.001*ActualVel*360)/60;
	return dupers;//  °/s
}

void Epos::Get_ActualVel(uint8_t vel_data[])
{
	TransmitPDO(Epos::NodeID,2,vel_data);
}

int32_t Epos::Get_ActualAcc()
{
	int32_t ActualVel_1=Get_ActualVel();
	delay_ms(2);
	int32_t ActualVel_2=Get_ActualVel();
	ActualAcc=(ActualVel_2-ActualVel_1)/2;
	return ActualAcc;
}

void Epos::Get_ActualCurrent(uint8_t cur_data[])
{
	SDO_Read(Epos::NodeID,0x30D1,0x02,cur_data);
}

BOOL Epos::IsHomePosition()
{
//	NMT_State_Change(0,NMT_Start_Node);
	
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
	ClearFault();
	
	InitPPM();
	
	EnableDevice();
	
	Set_ProfileVel(profile_vel);
	Set_ProfileAcc(profile_acc);
	Set_ProfileDec(profile_dec);
	Set_TargetPosition(IsAbsolute,target_position);

	if(m_bIsAbsolute)
		WriteControlword(absolute_immediate_movement);
	else
		WriteControlword(relative_immediate_movement);
}

void Epos::MoveToPosition(uint8_t IsAbsolute,int32_t target_position)
{
	EnableDevice();	
	
	Set_TargetPosition(IsAbsolute,target_position);

	if(m_bIsAbsolute)
		WriteControlword(absolute_immediate_movement);
	else
		WriteControlword(relative_immediate_movement);
	
}

void Epos::MoveToPosition(Uint32 profile_vel,Uint32 profile_acc,Uint32 profile_dec,int32_t *posList, uint8_t len)
{	
	//InitPPM();
	
	for(int i=0;i<len;i++)
	{
		MoveToPosition(profile_vel,profile_acc,profile_dec,1,posList[i]);
		delay_ms(100);
	}
}

/*写入控制字*/
void Epos::WriteControlword(uint16_t controlword)
{
	while(ReceivePDO(Epos::NodeID,1,controlword)!=1)//RxPDO1传递控制字
	{delay_ms(2);}
}

/*读出状态字*/
uint16_t Epos::ReadStatusword()
{
	uint16_t statusword;
	uint8_t status_data[2];
	TransmitPDO(Epos::NodeID,1,status_data);
	statusword=(status_data[1]<<8)+status_data[0];
	print_data_hex(&UART1_Handler,(uint32_t)statusword);
	
	return statusword;
}

/*写入对象字典*/
void Epos::WriteObject(uint16_t Index,uint8_t SubIndex,uint32_t param)
{
	while(SDO_Write(Epos::NodeID,Index,SubIndex,param)!=1)
	{delay_ms(2);}
}

/*初始化轮廓位置控制模式PPM*/
void Epos::InitPPM()
{
	NMT_State_Change(NodeID,NMT_Start_Node);
	
	Set_Operation_Mode(PPM);
	
	println_str(&UART1_Handler,"Set PPM Mode");
}

/*初始化循环同步位置模式CSP*/
void Epos::InitCSP()
{
	NMT_State_Change(NodeID,NMT_Start_Node);
	
	Set_Operation_Mode(CSP);
	
	println_str(&UART1_Handler,"Set CSP Mode");
}

void Epos::EnableDevice()
{
	Sync_Send();
	
	WriteControlword(shutdown);
	
	Sync_Send();
	
	WriteControlword(switchon_and_enable);
}

void Epos::ClearFault()
{
	println_str(&UART1_Handler,"Clear Fault");
	WriteObject(0x1003,0x00,0);
}

