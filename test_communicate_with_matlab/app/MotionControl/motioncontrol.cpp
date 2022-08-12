#include "motioncontrol.h"

Epos motor_W1(1,PPM);
Epos motor_W2(2,PPM);
Epos motor_QB(3,PPM);
Epos motor_ZB(4,PPM);

void Wrist_Extension()
{
	println_str(&UART1_Handler,"The wrist is extending. ");
	if(!motor_W1.IsHomePosition())
		motor_W1.BackToHomePosition();
	if(!motor_W2.IsHomePosition())
		motor_W2.BackToHomePosition();
	
	motor_W1.MoveToPosition(3000,motor_W1.ProfileAcc,motor_W1.ProfileDec,1,72200);
	motor_W2.MoveToPosition(3000,motor_W2.ProfileAcc,motor_W2.ProfileDec,1,-72200);
}

void Wrist_Flextion()
{
	println_str(&UART1_Handler,"The wrist is flexing. ");
	if(!motor_W1.IsHomePosition())
		motor_W1.BackToHomePosition();
	if(!motor_W2.IsHomePosition())
		motor_W2.BackToHomePosition();
	
//	motor_W1.MoveToPosition(3000,motor_W1.ProfileAcc,motor_W1.ProfileDec,1,-72200);
//	motor_W2.MoveToPosition(3000,motor_W2.ProfileAcc,motor_W2.ProfileDec,1,72200);
	motor_W1.MoveToPosition(3000,motor_W1.ProfileAcc,motor_W1.ProfileDec,1,0);
	motor_W2.MoveToPosition(3000,motor_W2.ProfileAcc,motor_W2.ProfileDec,1,0);
}
	
void Wrist_Ulnar_Deviation()
{
	println_str(&UART1_Handler,"The Wrist Ulnar Deviation");
	if(!motor_W1.IsHomePosition())
		motor_W1.BackToHomePosition();
	if(!motor_W2.IsHomePosition())
		motor_W2.BackToHomePosition();
	
	motor_W1.WriteObject(0x1003,0x00,0);
	motor_W2.WriteObject(0x1003,0x00,0);
	
	motor_W1.MoveToPosition(3000,motor_W1.ProfileAcc,motor_W1.ProfileDec,1,42100);
	motor_W2.MoveToPosition(3000,motor_W2.ProfileAcc,motor_W2.ProfileDec,1,42100);
}

void Wrist_Radial_Deviation()
{
	println_str(&UART1_Handler,"The Wrist Radial Deviation");
	if(!motor_W1.IsHomePosition())
		motor_W1.BackToHomePosition();
	if(!motor_W2.IsHomePosition())
		motor_W2.BackToHomePosition();
	
	motor_W1.MoveToPosition(3000,motor_W1.ProfileAcc,motor_W1.ProfileDec,1,-84200);
	motor_W2.MoveToPosition(3000,motor_W2.ProfileAcc,motor_W2.ProfileDec,1,-84200);
}

void Motor_Reset()
{
	if(!motor_W1.IsHomePosition())
			motor_W1.BackToHomePosition();
	delay_ms(10);
		
	if(!motor_W2.IsHomePosition())
			motor_W2.BackToHomePosition();
	delay_ms(10);
	
	if(!motor_QB.IsHomePosition())
			motor_QB.BackToHomePosition();
	delay_ms(10);
}
