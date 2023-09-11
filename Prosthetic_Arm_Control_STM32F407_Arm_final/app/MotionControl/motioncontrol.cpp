#include "motioncontrol.h"

using namespace std;

//Epos motor_W1(1,PPM);
//Epos motor_W2(2,PPM);
//Epos motor_QB(3,PPM);
//Epos motor_ZB(4,PPM);
FuzzyPID Poseture_Adjustment;
ImpedanceModel Impedance_Adjustment;

void Wrist_Extension()
{
	println_str(&UART1_Handler,"The wrist is extending. ");
	Motor_Reset();
//	delay_ms(500);
	
	motor_W1.MoveToPosition(1,72200);
	motor_W2.MoveToPosition(1,-72200);
}

void Wrist_Flextion()
{
	println_str(&UART1_Handler,"The wrist is flexing. ");
	Motor_Reset();
	delay_ms(500);
	
	motor_W1.MoveToPosition(1,-72200);
	motor_W2.MoveToPosition(1,72200);
}
	
void Wrist_Ulnar_Deviation()
{
	println_str(&UART1_Handler,"The Wrist Ulnar Deviation");
	Motor_Reset();
	delay_ms(500);

	motor_W1.MoveToPosition(1,42100);
	motor_W2.MoveToPosition(1,42100);
}

void Wrist_Radial_Deviation()
{
	println_str(&UART1_Handler,"The Wrist Radial Deviation");
	Motor_Reset();
	delay_ms(500);
	
	motor_W1.MoveToPosition(1,-84200);
	motor_W2.MoveToPosition(1,-84200);
}

void Joint_Motor_Init(void)
{
	Motor_W1(0);
	Motor_W2(0);
	Motor_QB(0);
	Motor_ZB(0);
}

void Motor_Reset()
{
	motor_W1.BackToHomePosition();
	motor_W2.BackToHomePosition();
	motor_QB.BackToHomePosition();
}

void Motor_W1(int32_t ReceiveCommand)
{
	motor_W1.MoveToPosition(3000,10000,10000,0,ReceiveCommand);
}

void Motor_W2(int32_t ReceiveCommand)
{
	motor_W2.MoveToPosition(3000,10000,10000,0,ReceiveCommand);
}

void Motor_QB(int32_t ReceiveCommand)
{
	ReceiveCommand=ReceiveCommand/((360.0/GearRatio)/(1000.0*Harmonic));
	motor_QB.MoveToPosition(3000,10000,10000,1,ReceiveCommand);
}

void Motor_ZB(int32_t ReceiveCommand)
{
	ReceiveCommand=(ReceiveCommand/3.6)*4000.0;
	motor_ZB.MoveToPosition(3000,10000,10000,1,ReceiveCommand);
}
//void Motor_W1(int16_t ReceiveCommand)
//{
//	motor_W1.MoveToPosition(3000,10000,10000,1,ReceiveCommand);
//}

//void Motor_W2(int16_t ReceiveCommand)
//{
//	motor_W2.MoveToPosition(3000,10000,10000,1,ReceiveCommand);
//}

//void Motor_QB(int16_t ReceiveCommand)
//{
//	motor_QB.MoveToPosition(3000,10000,10000,1,ReceiveCommand);
//}

void WristPositionControl(float PositionError[])
{
	float PositionErrorDifference[3];
	u8 i;
	int32_t motorinc[3]={0};
	
	for(i=0;i<3;i++)
	{
			PositionErrorPpre[i]=PositionErrorPre[i];
			PositionErrorPre[i]=PositionError[i];//假肢腕上一次的位姿偏差
			PositionErrorDifference[i]=PositionError[i]-PositionErrorPre[i];//当前位姿偏差和上次位姿偏差的变化（差值）
	}
	
	NMT_State_Change(0,NMT_Start_Node);
	
	if(abs(PositionError[0])>1||abs(PositionError[1])>1)//x,y轴角度偏差纠正
	{
		int32_t motorincx=Poseture_Adjustment.FuzzyPIDcontroller(120,-120, 20,-20, 100, 400, PositionError[0],PositionErrorDifference[0],20,30,15,20,PositionErrorPre[0],PositionErrorPpre[0]);
		int32_t motorincy=Poseture_Adjustment.FuzzyPIDcontroller(120,-120, 20,-20, 100, 400, PositionError[1],PositionErrorDifference[1],20,30,15,20,PositionErrorPre[1],PositionErrorPpre[1]);
		
		if(PositionError[0]>0&&PositionError[1]>0)
		{
			motorinc[0]=(abs(motorincx)>=abs(motorincy))?(-motorincx):(-motorincy);
			motorinc[1]=-motorincx+motorincy;
		}
		else if (PositionError[0]>0&&PositionError[1]<0)
		{
			motorinc[0]=-motorincx+motorincy;
			motorinc[1]=(abs(motorincx)>=abs(motorincy))?(-motorincx):(-motorincy);
		}
		else if(PositionError[0]<0&&PositionError[1]>0)
		{
			motorinc[0]=motorincx-motorincy;
			motorinc[1]=(abs(motorincx)>=abs(motorincy))?(motorincx):(motorincy);
		}
		else
		{
			motorinc[0]=(abs(motorincx)>=abs(motorincy))?(motorincx):(motorincy);
			motorinc[1]=motorincx-motorincy;
		}
		motor_W1.Get_ActualPos();
		motor_W2.Get_ActualPos();
		LIMIT(motorinc[0],-154800-motor_W1.ActualPos,154800-motor_W1.ActualPos);
		LIMIT(motorinc[1],-154800-motor_W2.ActualPos,154800-motor_W2.ActualPos);
		motor_W1.MoveToPosition(0,motorinc[0]);
		motor_W2.MoveToPosition(0,motorinc[1]);
		motorinc[0]=0;
		motorinc[1]=0;
	}
	
	if(abs(PositionError[2])>1)//z轴角度偏差纠正
	{
		motorinc[2]=Poseture_Adjustment.FuzzyPIDcontroller(120,-120, 20,-20, -50, -100, PositionError[2],PositionErrorDifference[2],-10,-20,-10,-20,PositionErrorPre[2],PositionErrorPpre[2]);
//		motor_QB.Get_ActualPos();
//		LIMIT(motorinc[2],-154800-motor_QB.ActualPos,154800-motor_QB.ActualPos);
		LIMIT(motorinc[2],-154800,154800);
//		motor_QB.MoveToPosition(0,-motorinc[2]);//√
		Motor_QB(0);
		motorinc[2]=0;
	}
}

void Elbow_CompliantControl(float ReceiveCommand)
{
//	motor_ZB.InitPPM();
//	
//	motor_ZB.EnableDevice();
//	int torque_R=motor_ZB.Get_AcutalTor();
//	printf("The real torque:%d\n ",torque_R);

////	int torque_D=Impedance_Adjustment.Get_DesireTorque();
////	float M=Impedance_Adjustment.Get_MASS();
////	float B=Impedance_Adjustment.Get_Damp();
////	float K=Impedance_Adjustment.Get_Stiffness();
////	Uint32 qd_acc=Impedance_Adjustment.Get_DesireAcc();
////	Uint32 qd_vel=Impedance_Adjustment.Get_DesireVel();
////	
//	Uint32 q_acc=motor_ZB.Get_ActualAcc();	
//	printf("The real accelerate:%d\n ",q_acc);
//	
//	int32_t q=motor_ZB.Get_ActualPos();
//	printf("The real position:%d\n ",q);
//	
//	Uint32 q_vel=motor_ZB.Get_ActualVel();
//	printf("The real velocity:%d\n ",q_vel);
//	if(abs(torque_R-torque_D)>1)
//	{
//		Uint32 qd=((torque_R-torque_D)-M*(q_acc-qd_acc)-B*(q_vel-qd_vel))/(-K)+q;
//		
//	}
//	else
//	{
		ReceiveCommand=ReceiveCommand*(278*3*3);
		motor_ZB.MoveToPosition(3000,10000,10000,1,ReceiveCommand);
		double torque_R=motor_ZB.Get_AcutalTor();
		printf("The real torque:%.7f\n ",torque_R);

		Uint32 q_acc=motor_ZB.Get_ActualAcc();	
		printf("The real accelerate:%d\n ",q_acc);
		
		int32_t q=motor_ZB.Get_ActualPos();
		printf("The real position:%d\n ",q);
		
		double q_vel=motor_ZB.Get_ActualVel();
		printf("The real velocity:%.7f\n ",q_vel);
//	}
}

void WristPostureControl(int32_t SendCommand[])
{
	float WristPos[3]={0};
	float delta_L[2]={0};
	static float CurErr[2]={0};
	static float PreErr[2]={0};
	int32_t x,y;
	int32_t M1,M2;
	double  kp_x=0.25,kd_x=0.3,kp_y=0.25,kd_y=0.3;
//	double  kp_x=1.5,kd_x=1,kp_y=1.5,kd_y=1;
	if(SendCommand[0]>=-30&&SendCommand[0]<=30&&SendCommand[2]<=30&&SendCommand[2]>=-30)
	{
			WristPoseEstimate();
			RPY(WristPos,SPlatform,MPlatform);//解算腕关节动平台相对于静平台的姿态(输出是弧度)
			x=0;y=0;
			CurErr[0]=SendCommand[0]-(WristPos[0]*180/Pi);//PD控制
			CurErr[1]=SendCommand[2]-(WristPos[1]*180/Pi);
		
			PreErr[0]-=CurErr[0];
			PreErr[1]-=CurErr[1];
//			SendCommand[0]=kp_x*CurErr[0]+kd_x*PreErr[0];
//			SendCommand[2]=kp_y*CurErr[1]+kd_y*PreErr[1];
			SendCommand[0]+=(kp_x*CurErr[0]+kd_x*PreErr[0]);
			SendCommand[2]+=(kp_y*CurErr[1]+kd_y*PreErr[1]);	
			if(abs(CurErr[0])>=3||abs(CurErr[1])>=3)
			{				
#if defined (Deviation_X) && (Deviation_X)
				/*绳子的缩短量计算*/
				delta_L[0]=sqrt(2536*(1-cos((betax-SendCommand[0])*Pi/180)))-sqrt(2536*(1-cos((betax*Pi/180)-WristPos[0])));//IMU的轴转动/尺偏桡偏运动/左右方向
				delta_L[1]=sqrt(2320*(1-cos((betay-SendCommand[2])*Pi/180)))-sqrt(2320*(1-cos((betay*Pi/180)-WristPos[1])));//IMU的Y轴转动/屈曲伸展运动/前后方向
				/*姿态引起的电机行程的绝对值*/
				x=((abs(delta_L[0])/R_CL)*(180.0/Pi))/((360.0/GearRatio)/(CountsPerTurn*Harmonic));
				y=((abs(delta_L[1])/R_CL)*(180.0/Pi))/((360.0/GearRatio)/(CountsPerTurn*Harmonic));
						
				/*电机行程计算*/
				M1=0;M2=0;
				if(delta_L[0]<0)//动平台向左运动:左偏变大/右偏变小
				{
					M1+=x;
					M2+=x;
				}
				if(delta_L[0]>0)//动平台向右运动:左偏变小/右偏变大
				{
					M1-=x;
					M2-=x;
				}
				if(delta_L[1]>0)//动平台向后运动：后偏变大/前偏变小
				{
					M1+=y;
					M2-=y;
				}
				if(delta_L[1]<0)//动平台向前运动：后偏变小/前偏变大
				{
					M1-=y;
					M2+=y;
				}
#else
				/*绳子的缩短量计算*/
				delta_L[0]=sqrt(2320*(1-cos((betax-SendCommand[0])*Pi/180)))-sqrt(2320*(1-cos((betax*Pi/180)-WristPos[0])));//IMU的X轴转动/尺偏桡偏运动/左右方向
				delta_L[1]=sqrt(2536*(1-cos((betay-SendCommand[2])*Pi/180)))-sqrt(2536*(1-cos((betay*Pi/180)-WristPos[1])));//IMU的Y轴转动/屈曲伸展运动/前后方向
				/*姿态引起的电机行程的绝对值*/
				x=((abs(delta_L[0])/R_CL)*(180.0/Pi))/((360.0/GearRatio)/(CountsPerTurn*Harmonic));
				y=((abs(delta_L[1])/R_CL)*(180.0/Pi))/((360.0/GearRatio)/(CountsPerTurn*Harmonic));
						
				/*电机行程计算*/
				M1=0;M2=0;
				if(delta_L[1]<0)//动平台向左运动:左偏变大/右偏变小
				{
					M1-=y;
					M2-=y;
				}
				if(delta_L[1]>0)//动平台向右运动:左偏变小/右偏变大
				{
					M1+=y;
					M2+=y;
				}
				if(delta_L[0]>0)//动平台向后运动：后偏变大/前偏变小
				{
					M1-=x;
					M2+=x;
				}
				if(delta_L[0]<0)//动平台向前运动：后偏变小/前偏变大
				{
					M1+=x;
					M2-=x;
				}
#endif
				Motor_W1(M1);
				Motor_W2(M2);
				PreErr[0]=CurErr[0];//PD控制
				PreErr[1]=CurErr[1];
			}
	}
}
