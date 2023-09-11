#include "resistance.h"

int32_t data_current_pos_original;
uint8_t register_current_torque(uint8_t torque , int32_t * data_current_pos);


uint8_t detect_torque_outside(double actual_torque, double threahold_torque)
{
	if ( actual_torque > threahold_torque || 
			 actual_torque < ((-1)*threahold_torque) )
	{
		
		register_current_torque(actual_torque , &data_current_pos_original);
		
		return RET_TRUE;
	} 
	else 
	{
		return RET_FALSE;
	}
	
}

uint8_t register_current_torque(uint8_t torque , int32_t * data_current_pos)
{
		
		return RET_TRUE;
}
	
void kick_off_retreat(uint8_t data_current_pos_original_tmp);  

void kick_off_recover(uint8_t data_current_pos_original_tmp);
