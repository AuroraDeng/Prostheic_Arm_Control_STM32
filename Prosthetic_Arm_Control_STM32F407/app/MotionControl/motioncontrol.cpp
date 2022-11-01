#include "motioncontrol.h"

Epos motor_W1(1,PPM);
Epos motor_W2(2,PPM);
Epos motor_QB(3,PPM);
Epos motor_ZB(4,PPM);
FuzzyPID Poseture_Adjustment;

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

void HelloWorld()
{
	for(int i=4;i>0;i--)
	{
		Wrist_Radial_Deviation();
		delay_ms(1000);
		Wrist_Ulnar_Deviation();
		delay_ms(1000);
	}
}
void Motor_Reset()
{
	motor_W1.BackToHomePosition();
	motor_W2.BackToHomePosition();
	motor_QB.BackToHomePosition();
}

void Motor_W1(int32_t ReceiveCommand)
{
	if(!motor_W1.IsHomePosition())
	{
		motor_W1.BackToHomePosition();
	}
	motor_W1.MoveToPosition(1,ReceiveCommand);
}

void Motor_W2(int32_t ReceiveCommand)
{
	if(!motor_W2.IsHomePosition())
	{
		motor_W2.BackToHomePosition();
	}
	motor_W2.MoveToPosition(1,ReceiveCommand);
}

void Motor_QB(int32_t ReceiveCommand)
{
	if(!motor_QB.IsHomePosition())
	{
		motor_QB.BackToHomePosition();
	}
	motor_QB.MoveToPosition(1,ReceiveCommand);
}

void WristPositionControl(float PositionError[])
{
	float PositionErrorDifference[3];
	u8 i;
	int32_t motorinc;
	
	for(i=0;i<3;i++)
	{
			PositionErrorPpre[i]=PositionErrorPre[i];
			PositionErrorPre[i]=PositionError[i];//��֫����һ�ε�λ��ƫ��
			PositionErrorDifference[i]=PositionError[i]-PositionErrorPre[i];//��ǰλ��ƫ����ϴ�λ��ƫ��ı仯����ֵ��
	}
	
	if(abs(PositionError[0])>1)//x��Ƕ�ƫ�����
	{
		motorinc=Poseture_Adjustment.FuzzyPIDcontroller(120,-120, 20,-20, -50, -100, PositionError[i],PositionErrorDifference[i],-10,-20,-10,-20,PositionErrorPre[i],PositionErrorPpre[i]);
		Motor_W1(motorinc);
		Motor_W2(motorinc);
	}
	
	if(abs(PositionError[1])>1)//y��Ƕ�ƫ�����
	{
		motorinc=Poseture_Adjustment.FuzzyPIDcontroller(120,-120, 20,-20, -50, -100, PositionError[i],PositionErrorDifference[i],-10,-20,-10,-20,PositionErrorPre[i],PositionErrorPpre[i]);
		Motor_W1(motorinc);
		Motor_W2(-motorinc);
	}
	
	if(abs(PositionError[2])>1)//z��Ƕ�ƫ�����
	{
		motorinc=Poseture_Adjustment.FuzzyPIDcontroller(120,-120, 20,-20, -50, -100, PositionError[i],PositionErrorDifference[i],-10,-20,-10,-20,PositionErrorPre[i],PositionErrorPpre[i]);
		Motor_QB(motorinc);
	}
}