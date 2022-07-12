#include "motor_test.h"

Epos motor_test(3,PPM);

void test(int32_t target_position)
{
	println_str(&UART1_Handler,"The motor is testing.");
	
	if(!motor_test.IsHomePosition())
		motor_test.BackToHomePosition();
	
	println_str(&UART1_Handler,"The motor is at home position.");	
	motor_test.MoveToPosition(3000,motor_test.ProfileAcc,motor_test.ProfileDec,1,target_position);
}