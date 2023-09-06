/*

	2.??????,??????????????????		2.1,???????????,????????2.2.1?????????)
			| (ret =) uint8_t detect_torque_outside(motor_ZB.Get_AcutalTor(),THREHOLD_TORQUE);
			| if( 1 == ret ) register_current_torque(data_ret[38],(int) data_current_pos);
			| kick_off_retreat(data_current_pos);                    //?????????,??????,????????,???			
		2.2,???????????,????????????????2.2.1?????????)
			| kick_off_recover(data_current_pos);                   //???????????,?????????,??????,?????kick_off)retreat
								
*/


#ifndef __RESISTANCE_H
#define __RESISTANCE_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "can.h"
#include "pdo.h"
#include "sdo.h"
#include "usart_print.h"
#include "delay.h"
#include "sys.h"
#include "step.h"

//------------------------------------------------------------------------
//define variables  定义变量类型
//------------------------------------------------------------------------
#define LIXIN_ON  1
#define LIXIN_OFF 0

#define RET_TRUE  1
#define RET_FALSE 0

#define THREHOLD_TORQUE 0.0200

typedef uint8_t ret_type;




#ifdef __cplusplus
extern "C" {
#endif
	
uint8_t detect_torque_outside(double actual_torque, double threahold_torque);
	
uint8_t register_current_torque(uint8_t torque , int32_t * data_current_pos);
	
void kick_off_retreat(uint8_t data_current_pos_original_tmp);  

void kick_off_recover(uint8_t data_current_pos_original_tmp);


#ifdef __cplusplus
}
#endif
#endif
