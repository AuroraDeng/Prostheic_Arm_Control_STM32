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
	
		float M,B,K;//��ؽ��迹����
		float torque_d,torque_r;//��ؽ�Ŀ�����ء���ؽ�ʵ������
		float q_acc,q_vel,q;//��ؽ�ʵ�ʽǼ��ٶȡ�ʵ�ʽ��ٶȺ�ʵ�ʽ�λ��
		float qd_acc,qd_vel,qd;//��ؽ�Ŀ��Ǽ��ٶȡ�Ŀ����ٶȺ�Ŀ���λ��
};
	
#ifdef __cplusplus
}
#endif

#endif