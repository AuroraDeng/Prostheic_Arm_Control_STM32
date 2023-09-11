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
		float M,B,K;//��ؽ��迹����
		float torque_d;//��ؽ�Ŀ�����ء���ؽ�ʵ������
		int32_t qd,qd_vel,qd_acc;//��ؽ�Ŀ���λ�ơ�Ŀ����ٶȡ�Ŀ��Ǽ��ٶ�
	
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