#include "motioncontrol.h"

Epos motor_W1(1,PPM);
Epos motor_W2(2,PPM);
Epos motor_QB(3,PPM);
Epos motor_ZB(4,PPM);

void Wrist_Extension()
{
	println_str(&UART1_Handler,"The wrist is extending. ");
	Motor_Reset();
	delay_ms(500);
	
	motor_W1.MoveToPosition(3000,motor_W1.ProfileAcc,motor_W1.ProfileDec,1,72200);
	motor_W2.MoveToPosition(3000,motor_W2.ProfileAcc,motor_W2.ProfileDec,1,-72200);
}

void Wrist_Flextion()
{
	println_str(&UART1_Handler,"The wrist is flexing. ");
	Motor_Reset();
	delay_ms(500);
	
	motor_W1.MoveToPosition(3000,motor_W1.ProfileAcc,motor_W1.ProfileDec,1,-72200);
	motor_W2.MoveToPosition(3000,motor_W2.ProfileAcc,motor_W2.ProfileDec,1,72200);
}
	
void Wrist_Ulnar_Deviation()
{
	println_str(&UART1_Handler,"The Wrist Ulnar Deviation");
	Motor_Reset();
	delay_ms(500);

	
	motor_W1.MoveToPosition(3000,motor_W1.ProfileAcc,motor_W1.ProfileDec,1,42100);
	motor_W2.MoveToPosition(3000,motor_W2.ProfileAcc,motor_W2.ProfileDec,1,42100);
}

void Wrist_Radial_Deviation()
{
	println_str(&UART1_Handler,"The Wrist Radial Deviation");
	Motor_Reset();
	delay_ms(500);
	
	motor_W1.MoveToPosition(3000,motor_W1.ProfileAcc,motor_W1.ProfileDec,1,-84200);
	motor_W2.MoveToPosition(3000,motor_W2.ProfileAcc,motor_W2.ProfileDec,1,-84200);
}

void HelloWorld()
{
	for(int i=4;i>0;i--)
	{
		Wrist_Radial_Deviation();
		delay_ms(500);
		Wrist_Ulnar_Deviation();
		delay_ms(500);
	}
}
void Motor_Reset()
{
			motor_W1.BackToHomePosition();
			motor_W2.BackToHomePosition();
			motor_QB.BackToHomePosition();
}
