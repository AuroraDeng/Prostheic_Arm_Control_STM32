#ifndef __KAERMAN_H
#define __KAERMAN_H

#ifdef __cplusplus
extern "C" {
#endif
	
#define Pi 	3.1415926
//��������������
#define dt           0.012
#define R_angle      0.5
#define Q_angle      0.001
#define Q_gyro       0.003//Խ��Խ�ͺ�


class IMU
{
	private:
		void Kalman_Filter_X(float Accel,float Gyro);
		void Kalman_Filter_Y(float Accel,float Gyro);
		void Kalman_Filter_Z(float Accel, float Gyro);
		//����������
		char  C_0 = 1;
		float Q_bias_x, Q_bias_y, Q_bias_z;
		float Angle_err_x, Angle_err_y, Angle_err_z;
		float PCt_0, PCt_1, E;
		float K_0, K_1, t_0, t_1;
		float Pdot[4] = { 0,0,0,0 };
		float PP[2][2] = { { 1, 0 },{ 0, 1 } };
		//double KalmanFilter(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R);

	public:
		short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
		short gyrox,gyroy,gyroz;	//������ԭʼ����

		float hN_roll = 0; 
		float hN_Pitch = 0; 
		float hN_Yaw = 0; 

		float Accel_x;	     //X����ٶ�ֵ�ݴ�
		float Accel_y;	     //Y����ٶ�ֵ�ݴ�
		float Accel_z;	     //Z����ٶ�ֵ�ݴ�

		float Gyro_x;		 //X�������������ݴ�
		float Gyro_y;    //Y�������������ݴ�
		float Gyro_z;		 //Z�������������ݴ�

		//float Angle_gy;    //�ɽ��ٶȼ������б�Ƕ�
		float Angle_x_temp;  //�ɼ��ٶȼ����x��б�Ƕ�
		float Angle_y_temp;  //�ɼ��ٶȼ����y��б�Ƕ�
		float Angle_z_temp;

		float Angle_X_Final; //X������б�Ƕ�
		float Angle_Y_Final; //Y������б�Ƕ�
		float Angle_Z_Final; //Z������б�Ƕ�
		
		void Angle_Calcu(void);
};

//void yijiehubu_P(float angle_m, float gyro_m);
//void erjiehubu_P(float angle_m, float gyro_m);
//void Erjielvbo(float angle_m,float gyro_m);
void CopeMPData(unsigned char ucData);
void CopeSPData(unsigned char ucData);
extern IMU MPlatform;
extern IMU SPlatform;

#ifdef __cplusplus
}
#endif
#endif
