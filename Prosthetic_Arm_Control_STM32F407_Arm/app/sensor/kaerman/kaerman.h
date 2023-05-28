#ifndef __KAERMAN_H
#define __KAERMAN_H

#ifdef __cplusplus
extern "C" {
#endif
	
#define Pi 	3.1415926
//卡尔曼噪声参数
#define dt           0.012
#define R_angle      0.5
#define Q_angle      0.001
#define Q_gyro       0.003//越大越滞后


class IMU
{
	private:
		void Kalman_Filter_X(float Accel,float Gyro);
		void Kalman_Filter_Y(float Accel,float Gyro);
		void Kalman_Filter_Z(float Accel, float Gyro);
		//卡尔曼参数
		char  C_0 = 1;
		float Q_bias_x, Q_bias_y, Q_bias_z;
		float Angle_err_x, Angle_err_y, Angle_err_z;
		float PCt_0, PCt_1, E;
		float K_0, K_1, t_0, t_1;
		float Pdot[4] = { 0,0,0,0 };
		float PP[2][2] = { { 1, 0 },{ 0, 1 } };
		//double KalmanFilter(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R);

	public:
		short aacx,aacy,aacz;		//加速度传感器原始数据
		short gyrox,gyroy,gyroz;	//陀螺仪原始数据

		float hN_roll = 0; 
		float hN_Pitch = 0; 
		float hN_Yaw = 0; 

		float Accel_x;	     //X轴加速度值暂存
		float Accel_y;	     //Y轴加速度值暂存
		float Accel_z;	     //Z轴加速度值暂存

		float Gyro_x;		 //X轴陀螺仪数据暂存
		float Gyro_y;    //Y轴陀螺仪数据暂存
		float Gyro_z;		 //Z轴陀螺仪数据暂存

		//float Angle_gy;    //由角速度计算的倾斜角度
		float Angle_x_temp;  //由加速度计算的x倾斜角度
		float Angle_y_temp;  //由加速度计算的y倾斜角度
		float Angle_z_temp;

		float Angle_X_Final; //X最终倾斜角度
		float Angle_Y_Final; //Y最终倾斜角度
		float Angle_Z_Final; //Z最终倾斜角度
		
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
