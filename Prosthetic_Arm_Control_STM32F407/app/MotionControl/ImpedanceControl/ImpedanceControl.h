#ifndef __IMPEDANCECONTROL_H
#define __IMPEDANCECONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

class ImpedanceModel
{
	public:
		ImpedanceModel();
		~ImpedanceModel();
		void Set_Impedance_Parameter();
	
		float M,B,K;//肘关节阻抗参数
		float torque_d,torque_r;//肘关节目标力矩、肘关节实际力矩
		float q_acc,q_vel,q;//肘关节实际角加速度、实际角速度和实际角位移
		float qd_acc,qd_vel,qd;//肘关节目标角加速度、目标角速度和目标角位移
};
	
#ifdef __cplusplus
}
#endif

#endif