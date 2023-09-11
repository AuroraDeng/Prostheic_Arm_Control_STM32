#ifndef __IMPEDANCECONTROL_H
#define __IMPEDANCECONTROL_H

#ifdef __cplusplus
extern "C" {
#endif
#include "EPOS.h"	
extern Epos motor_ZB;
	
class ImpedanceModel
{	
	private:
		float M,B,K;//肘关节阻抗参数
		float torque_d;//肘关节目标力矩、肘关节实际力矩
		int32_t qd,qd_vel,qd_acc;//肘关节目标角位移、目标角速度、目标角加速度
	
	public:
		ImpedanceModel();
		void Set_Impedance_Parameter();
		float Get_DesireTorque()const {return torque_d;}
		float Get_MASS()const {return M;}
		float Get_Damp()const {return B;}
		float Get_Stiffness()const {return K;}
		int32_t Get_DesireAcc()const {return qd_acc;}
		int32_t Get_DesireVel()const {return qd_vel;}
};
	
#ifdef __cplusplus
}
#endif

#endif