#ifndef __FUZZYPID_H
#define __FUZZYPID_H

#ifdef __cplusplus
extern "C" {
#endif

class FuzzyPID
{
	public:
    FuzzyPID();
    ~FuzzyPID();
		void Get_grad_membership(float erro,float erro_c);//����e��de/dt�����ȼ��㺯��///
		void GetSumGrad();//��ȡ�������kp,ki,kd����������/
		void GetOUT();//�����������kp,kd,ki��Ӧ����ֵ//
		float FuzzyPIDcontroller(float e_max, float e_min, float ec_max, float ec_min, float kp_max, float kp_min, float erro, float erro_c,float ki_max,float ki_min,float kd_max,float kd_min,float erro_pre,float errp_ppre);
		float Quantization(float maximum,float minimum,float x);//����ӳ�亯��
		float Inverse_quantization(float maximum, float minimum, float qvalues);
		int changeOut(double out,double sss);
	
		const int  num_area = 8; //�����������
		float e_membership_values[7]  = { -3,-2,-1,0,1,2,3 };//����E(Ŀ��λ����ʵ��λ�˵�ƫ��)������ֵ
    float ec_membership_values[7] = { -3,-2,-1,0,1,2,3 };//����dE/dt����ǰƫ����ϴ�ƫ��Ĳ�ֵ��������ֵ
		float kp_menbership_values[7] = { -3,-2,-1,0,1,2,3 };//�������kp������ֵ
    float ki_menbership_values[7] = { -3,-2,-1,0,1,2,3 }; //�������ki������ֵ
    float kd_menbership_values[7] = { -3,-2,-1,0,1,2,3 };  //�������kd������ֵ
		
		float kp,ki,kd;//PID����
		float qdetail_kp;               //����kp��Ӧ�����е�ֵ
    float qdetail_ki;               //����ki��Ӧ�����е�ֵ
    float qdetail_kd;               //����kd��Ӧ�����е�ֵ
    float qfuzzy_output;
    float detail_kp;                //�������kp
    float detail_ki;                //�������ki
    float detail_kd;                //�������kd
    float fuzzy_output;
		float erro,erro_c;//�����仯��
    float qerro;                    //����e��Ӧ�����е�ֵ
    float qerro_c;                  //����de/dt��Ӧ�����е�ֵ
    float errosum;
    float e_gradmembership[2];      //����e��������
    float ec_gradmembership[2];     //����de/dt��������
    int e_grad_index[2];            //����e�������ڹ���������
    int ec_grad_index[2];           //����de/dt�������ڹ���������
    float gradSums[7] = {0,0,0,0,0,0,0};
    float KpgradSums[7] = { 0,0,0,0,0,0,0 };   //�������kp�ܵ�������
    float KigradSums[7] = { 0,0,0,0,0,0,0 };   //�������ki�ܵ�������
    float KdgradSums[7] = { 0,0,0,0,0,0,0 };   //�������kd�ܵ�������
		
		
//		float integral,maxIntegral;//���֡������޷�
//		float fuzzy_output,maxOutput,minOutput;//���������޷�
	
		int NB = -3, NM = -2, NS = -1, ZO = 0, PS = 1, PM = 2, PB = 3; //��������ֵ

		int  Kp_rule_list[7][7] = { {PB,PB,PB,PB,PM,ZO,ZO},     //kp�����
                                {PB,PB,PB,PM,PM,ZO,ZO},
                                {PB,PM,PM,PS,ZO,NS,NM},
                                {PM,PM,PS,ZO,NS,NM,NM},
                                {PS,PS,ZO,NM,NM,NM,NB},
                                {ZO,ZO,ZO,NM,NB,NB,NB},
                                {ZO,NS,NB,NB,NB,NB,NB} };

		int  Ki_rule_list[7][7] = { {PB,PB,PB,PB,PM,ZO,ZO},     //kp�����
                                {PB,PB,PB,PM,PM,ZO,ZO},
                                {PB,PM,PM,PS,ZO,NS,NM},
                                {PM,PM,PS,ZO,NS,NM,NM},
                                {PS,PS,ZO,NM,NM,NM,NB},
                                {ZO,ZO,ZO,NM,NB,NB,NB},
                                {ZO,NS,NB,NB,NB,NB,NB} };

		int  Kd_rule_list[7][7] = { {PB,PB,PB,PB,PM,ZO,ZO},     //kp�����
                                {PB,PB,PB,PM,PM,ZO,ZO},
                                {PB,PM,PM,PS,ZO,NS,NM},
                                {PM,PM,PS,ZO,NS,NM,NM},
                                {PS,PS,ZO,NM,NM,NM,NB},
                                {ZO,ZO,ZO,NM,NB,NB,NB},
                                {ZO,NS,NB,NB,NB,NB,NB} };
};
	
#ifdef __cplusplus
}
#endif

#endif