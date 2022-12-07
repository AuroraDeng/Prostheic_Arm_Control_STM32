#include "ImpedanceControl.h"

ImpedanceModel::ImpedanceModel()
{
		M=10;
		B=2;
		K=50;
		torque_d=1000;
		torque_r=0;
		q_acc=0;
		q_vel=0;
		q=0;
		qd_acc=1000;
		qd_vel=3000;
		qd=0;
}
		
ImpedanceModel::~ImpedanceModel()
{
	
}

void ImpedanceModel::Set_Impedance_Parameter()
{

}

