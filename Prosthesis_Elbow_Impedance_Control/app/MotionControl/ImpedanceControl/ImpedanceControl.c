#include "ImpedanceControl.h"
#include "motioncontrol.h"

ImpedanceModel::ImpedanceModel()
{
		M=10;
		B=2;
		K=50;
		torque_d=1;
		qd_acc=1000;
		qd_vel=3000;
		qd=0;
}

void ImpedanceModel::Set_Impedance_Parameter()
{

}